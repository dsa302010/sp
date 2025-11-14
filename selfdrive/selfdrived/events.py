#!/usr/bin/env python3
import math
import os
from cereal import log, car
import cereal.messaging as messaging
from openpilot.common.constants import CV
from openpilot.common.git import get_short_branch
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.locationd.calibrationd import MIN_SPEED_FILTER
from openpilot.system.micd import SAMPLE_RATE, SAMPLE_BUFFER
from openpilot.selfdrive.ui.feedback.feedbackd import FEEDBACK_MAX_DURATION
from openpilot.sunnypilot.selfdrive.selfdrived.events_base import EventsBase, Priority, ET, Alert, \
  NoEntryAlert, SoftDisableAlert, UserSoftDisableAlert, ImmediateDisableAlert, EngagementAlert, NormalPermanentAlert, \
  StartupAlert, AlertCallbackType, wrong_car_mode_alert
AlertSize = log.SelfdriveState.AlertSize
AlertStatus = log.SelfdriveState.AlertStatus
VisualAlert = car.CarControl.HUDControl.VisualAlert
AudibleAlert = car.CarControl.HUDControl.AudibleAlert
EventName = log.OnroadEvent.EventName
# 获取事件名
EVENT_NAME = {v: k for k, v in EventName.schema.enumerants.items()}
class Events(EventsBase):
  def __init__(self):
    super().__init__()
    self.event_counters = dict.fromkeys(EVENTS.keys(), 0)
  def get_events_mapping(self) -> dict[int, dict[str, Alert | AlertCallbackType]]:
    return EVENTS
  def get_event_name(self, event: int):
    return EVENT_NAME[event]
  def get_event_msg_type(self):
    return log.OnroadEvent
# ********** 辅助函数 **********
def get_display_speed(speed_ms: float, metric: bool) -> str:
  speed = int(round(speed_ms * (CV.MS_TO_KPH if metric else CV.MS_TO_MPH)))
  unit = '公里/小时' if metric else '英里/小时'
  return f"{speed} {unit}"
# ********** 警告回调函数 **********
def soft_disable_alert(alert_text_2: str) -> AlertCallbackType:
  def func(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
    if soft_disable_time < int(0.5 / DT_CTRL):
      return ImmediateDisableAlert(alert_text_2)
    return SoftDisableAlert(alert_text_2)
  return func
def user_soft_disable_alert(alert_text_2: str) -> AlertCallbackType:
  def func(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
    if soft_disable_time < int(0.5 / DT_CTRL):
      return ImmediateDisableAlert(alert_text_2)
    return UserSoftDisableAlert(alert_text_2)
  return func
def startup_master_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
  branch = get_short_branch() # 确保启动时缓存分支名，避免延迟
  if "REPLAY" in os.environ:
    branch = "回放"
  return StartupAlert("警告：此分支未经过测试", branch, alert_status=AlertStatus.userPrompt)
def below_engage_speed_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
  return NoEntryAlert(f"需行驶超过 {get_display_speed(CP.minEnableSpeed, metric)} 才能启用")
def below_steer_speed_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
  return Alert(
    f"低于 {get_display_speed(CP.minSteerSpeed, metric)} 转向不可用",
    "",
    AlertStatus.userPrompt, AlertSize.small,
    Priority.LOW, VisualAlert.none, AudibleAlert.prompt, 0.4)
def calibration_incomplete_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
  first_word = '重新校准' if sm['liveCalibration'].calStatus == log.LiveCalibrationData.Status.recalibrating else '校准'
  return Alert(
    f"{first_word}进行中：{sm['liveCalibration'].calPerc:.0f}%",
    f"需行驶超过 {get_display_speed(MIN_SPEED_FILTER, metric)}",
    AlertStatus.normal, AlertSize.mid,
    Priority.LOWEST, VisualAlert.none, AudibleAlert.none, .2)
def audio_feedback_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
  duration = FEEDBACK_MAX_DURATION - ((sm['audioFeedback'].blockNum + 1) * SAMPLE_BUFFER / SAMPLE_RATE)
  return NormalPermanentAlert(
    "正在录制语音反馈",
    f"剩余 {round(duration)} 秒，再次按下可提前保存。",
    priority=Priority.LOW)
# *** 调试警告 ***
def out_of_space_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
  full_perc = round(100. - sm['deviceState'].freeSpacePercent)
  return NormalPermanentAlert("存储空间不足", f"{full_perc}% 已用")
def posenet_invalid_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
  mdl = sm['modelV2'].velocity.x[0] if len(sm['modelV2'].velocity.x) else math.nan
  err = CS.vEgo - mdl
  msg = f"速度误差：{err:.1f} 米/秒"
  return NoEntryAlert(msg, alert_text_1="Posenet 速度无效")
def process_not_running_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
  not_running = [p.name for p in sm['managerState'].processes if not p.running and p.shouldBeRunning]
  msg = ', '.join(not_running)
  return NoEntryAlert(msg, alert_text_1="进程未运行")
def comm_issue_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
  bs = [s for s in sm.data.keys() if not sm.all_checks([s, ])]
  msg = ', '.join(bs[:4]) # 不能在一行显示太多
  return NoEntryAlert(msg, alert_text_1="进程间通信异常")
def camera_malfunction_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
  all_cams = ('roadCameraState', 'driverCameraState', 'wideRoadCameraState')
  bad_cams = [s.replace('State', '') for s in all_cams if s in sm.data.keys() and not sm.all_checks([s, ])]
  return NormalPermanentAlert("摄像头故障", ', '.join(bad_cams))
def calibration_invalid_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
  rpy = sm['liveCalibration'].rpyCalib
  yaw = math.degrees(rpy[2] if len(rpy) == 3 else math.nan)
  pitch = math.degrees(rpy[1] if len(rpy) == 3 else math.nan)
  angles = f"重新安装设备（俯仰: {pitch:.1f}°, 偏航: {yaw:.1f}°)"
  return NormalPermanentAlert("校准无效", angles)
def paramsd_invalid_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
  if not sm['liveParameters'].angleOffsetValid:
    angle_offset_deg = sm['liveParameters'].angleOffsetDeg
    title = "检测到方向盘偏差"
    text = f"角度偏移过大（偏移: {angle_offset_deg:.1f}°)"
  elif not sm['liveParameters'].steerRatioValid:
    steer_ratio = sm['liveParameters'].steerRatio
    title = "方向盘传动比异常"
    text = f"转向系统可能异常（传动比: {steer_ratio:.1f}）"
  elif not sm['liveParameters'].stiffnessFactorValid:
    stiffness_factor = sm['liveParameters'].stiffnessFactor
    title = "轮胎刚性异常"
    text = f"请检查轮胎、胎压或对齐（刚性系数: {stiffness_factor:.1f}）"
  else:
    return NoEntryAlert("paramsd 临时错误")
  return NoEntryAlert(alert_text_1=title, alert_text_2=text)
def overheat_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
  cpu = max(sm['deviceState'].cpuTempC, default=0.)
  gpu = max(sm['deviceState'].gpuTempC, default=0.)
  temp = max((cpu, gpu, sm['deviceState'].memoryTempC))
  return NormalPermanentAlert("系统过热", f"{temp:.0f} °C")
def low_memory_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
  return NormalPermanentAlert("内存不足", f"{sm['deviceState'].memoryUsagePercent}% 已用")
def high_cpu_usage_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
  x = max(sm['deviceState'].cpuUsagePercent, default=0.)
  return NormalPermanentAlert("CPU 使用率过高", f"{x}% 已用")
def modeld_lagging_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
  return NormalPermanentAlert("驾驶模型延迟", f"{sm['modelV2'].frameDropPerc:.1f}% 帧丢失")
def joystick_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
  gb = sm['carControl'].actuators.accel / 4.
  steer = sm['carControl'].actuators.torque
  vals = f"油门: {round(gb * 100.)}%, 转向: {round(steer * 100.)}%"
  return NormalPermanentAlert("摇杆模式", vals)
def longitudinal_maneuver_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
  ad = sm['alertDebug']
  audible_alert = AudibleAlert.prompt if 'Active' in ad.alertText1 else AudibleAlert.none
  alert_status = AlertStatus.userPrompt if 'Active' in ad.alertText1 else AlertStatus.normal
  alert_size = AlertSize.mid if ad.alertText2 else AlertSize.small
  return Alert(ad.alertText1, ad.alertText2,
               alert_status, alert_size,
               Priority.LOW, VisualAlert.none, audible_alert, 0.2)
def personality_changed_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
  personality = str(personality).title()
  return NormalPermanentAlert(f"驾驶性格：{personality}", duration=1.5)
def invalid_lkas_setting_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
  text = "切换原厂 LKAS 开/关以启用"
  if CP.brand == "tesla":
    text = "切换至交通感知巡航控制以启用"
  elif CP.brand == "mazda":
    text = "启用原厂 LKAS 以启用"
  elif CP.brand == "nissan":
    text = "关闭原厂 LKAS 以启用"
  return NormalPermanentAlert("LKAS 设置无效", text)
EVENTS: dict[int, dict[str, Alert | AlertCallbackType]] = {
  # ********** 无警告的事件 **********
  EventName.stockFcw: {},
  EventName.actuatorsApiUnavailable: {},
  # ********** 所有状态下显示的警告 **********
  EventName.joystickDebug: {
    ET.WARNING: joystick_alert,
    ET.PERMANENT: NormalPermanentAlert("摇杆模式"),
  },
  EventName.longitudinalManeuver: {
    ET.WARNING: longitudinal_maneuver_alert,
    ET.PERMANENT: NormalPermanentAlert("纵向操控模式",
                                       "确保前方道路畅通"),
  },
  EventName.selfdriveInitializing: {
    ET.NO_ENTRY: NoEntryAlert("系统初始化中"),
  },
  EventName.startup: {
    ET.PERMANENT: StartupAlert("随时准备接管")
  },
  EventName.startupMaster: {
    ET.PERMANENT: startup_master_alert,
  },
  EventName.startupNoControl: {
    ET.PERMANENT: StartupAlert("行车记录仪模式"),
    ET.NO_ENTRY: NoEntryAlert("行车记录仪模式"),
  },
  EventName.startupNoCar: {
    ET.PERMANENT: StartupAlert("不支持的车辆：行车记录仪模式"),
  },
  EventName.startupNoSecOcKey: {
    ET.PERMANENT: NormalPermanentAlert("行车记录仪模式",
                                       "安全密钥不可用",
                                       priority=Priority.HIGH),
  },
  EventName.dashcamMode: {
    ET.PERMANENT: NormalPermanentAlert("行车记录仪模式",
                                       priority=Priority.LOWEST),
  },
  EventName.invalidLkasSetting: {
    ET.PERMANENT: invalid_lkas_setting_alert,
    ET.NO_ENTRY: NoEntryAlert("LKAS 设置无效"),
  },
  EventName.cruiseMismatch: {
    #ET.PERMANENT: ImmediateDisableAlert("openpilot 未能取消定速巡航"),
  },
  # openpilot 不识别车辆。切换为只读模式。可通过添加指纹解决。
  # 详见 https://github.com/commaai/openpilot/wiki/Fingerprinting
  EventName.carUnrecognized: {
    ET.PERMANENT: NormalPermanentAlert("行车记录仪模式",
                                       "车辆未识别",
                                       priority=Priority.LOWEST),
  },
  EventName.aeb: {
    ET.PERMANENT: Alert(
      "刹车！",
      "紧急制动：碰撞风险",
      AlertStatus.critical, AlertSize.full,
      Priority.HIGHEST, VisualAlert.fcw, AudibleAlert.none, 2.),
    ET.NO_ENTRY: NoEntryAlert("AEB：碰撞风险"),
  },
  EventName.stockAeb: {
    ET.PERMANENT: Alert(
      "刹车！",
      "原厂 AEB：碰撞风险",
      AlertStatus.critical, AlertSize.full,
      Priority.HIGHEST, VisualAlert.fcw, AudibleAlert.none, 2.),
    ET.NO_ENTRY: NoEntryAlert("原厂 AEB：碰撞风险"),
  },
  EventName.fcw: {
    ET.PERMANENT: Alert(
      "刹车！",
      "碰撞风险",
      AlertStatus.critical, AlertSize.full,
      Priority.HIGHEST, VisualAlert.fcw, AudibleAlert.warningSoft, 2.),
  },
  EventName.ldw: {
    ET.PERMANENT: Alert(
      "检测到车道偏离",
      "",
      AlertStatus.userPrompt, AlertSize.small,
      Priority.LOW, VisualAlert.ldw, AudibleAlert.prompt, 3.),
  },
  # ********** 仅在启用时显示的警告 **********
  EventName.steerTempUnavailableSilent: {
    ET.WARNING: Alert(
      "转向暂时不可用",
      "",
      AlertStatus.userPrompt, AlertSize.small,
      Priority.LOW, VisualAlert.steerRequired, AudibleAlert.prompt, 1.8),
  },
  EventName.preDriverDistracted: {
    ET.PERMANENT: Alert(
      "请注意",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .1),
  },
  EventName.promptDriverDistracted: {
    ET.PERMANENT: Alert(
      "请注意",
      "驾驶员分心",
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.MID, VisualAlert.steerRequired, AudibleAlert.promptDistracted, .1),
  },
  EventName.driverDistracted: {
    ET.PERMANENT: Alert(
      "立即接管！",
      "驾驶员分心",
      AlertStatus.critical, AlertSize.full,
      Priority.HIGH, VisualAlert.steerRequired, AudibleAlert.warningImmediate, .1),
  },
  EventName.preDriverUnresponsive: {
    ET.PERMANENT: Alert(
      "触摸方向盘：未检测到面部",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.steerRequired, AudibleAlert.none, .1),
  },
  EventName.promptDriverUnresponsive: {
    ET.PERMANENT: Alert(
      "触摸方向盘",
      "驾驶员无响应",
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.MID, VisualAlert.steerRequired, AudibleAlert.promptDistracted, .1),
  },
  EventName.driverUnresponsive: {
    ET.PERMANENT: Alert(
      "立即接管！",
      "驾驶员无响应",
      AlertStatus.critical, AlertSize.full,
      Priority.HIGH, VisualAlert.steerRequired, AudibleAlert.warningImmediate, .1),
  },
  EventName.manualRestart: {
    ET.WARNING: Alert(
      "接管控制",
      "手动恢复驾驶",
  AlertStatus.userPrompt, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .2),
  },
  EventName.resumeRequired: {
    ET.WARNING: Alert(
      "按恢复退出停车",
      "",
      AlertStatus.userPrompt, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .2),
  },
  EventName.belowSteerSpeed: {
    ET.WARNING: below_steer_speed_alert,
  },
  EventName.preLaneChangeLeft: {
    ET.WARNING: Alert(
      "安全时向左打方向开始变道",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .1),
  },
  EventName.preLaneChangeRight: {
    ET.WARNING: Alert(
      "安全时向右打方向开始变道",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .1),
  },
  EventName.laneChangeBlocked: {
    ET.WARNING: Alert(
      "盲区检测到车辆",
      "",
      AlertStatus.userPrompt, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.prompt, .1),
  },
  EventName.laneChange: {
    ET.WARNING: Alert(
      "正在变道",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .1),
  },
  EventName.steerSaturated: {
    ET.WARNING: Alert(
      "接管控制",
      "转向超出极限",
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.LOW, VisualAlert.steerRequired, AudibleAlert.promptRepeat, 2.),
  },
  # 风扇转速 >50% 但未旋转
  EventName.fanMalfunction: {
    ET.PERMANENT: NormalPermanentAlert("风扇故障", "疑似硬件问题"),
  },
  # 摄像头无帧输出
  EventName.cameraMalfunction: {
    ET.PERMANENT: camera_malfunction_alert,
    ET.SOFT_DISABLE: soft_disable_alert("摄像头故障"),
    ET.NO_ENTRY: NoEntryAlert("摄像头故障：请重启设备"),
  },
  # 摄像头帧率过低
  EventName.cameraFrameRate: {
    ET.PERMANENT: NormalPermanentAlert("摄像头帧率过低", "请重启设备"),
    ET.SOFT_DISABLE: soft_disable_alert("摄像头帧率过低"),
    ET.NO_ENTRY: NoEntryAlert("摄像头帧率过低：请重启设备"),
  },
  # 未使用
  EventName.locationdTemporaryError: {
    ET.NO_ENTRY: NoEntryAlert("locationd 临时错误"),
    ET.SOFT_DISABLE: soft_disable_alert("locationd 临时错误"),
  },
  EventName.locationdPermanentError: {
    ET.NO_ENTRY: NoEntryAlert("locationd 永久错误"),
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert("locationd 永久错误"),
    ET.PERMANENT: NormalPermanentAlert("locationd 永久错误"),
  },
  # openpilot 通过观察人类和自动驾驶时的转向输入来学习车辆参数：
  # - 转向比：转向齿条传动比
  # - 轮胎刚性：轮胎抓地力
  # - 角度偏移：大多数转向传感器直行时有偏移
  # 当这些值超出合理范围时触发此警告。可能由对齐不良或传感器数据错误引起。
  EventName.paramsdTemporaryError: {
    ET.NO_ENTRY: paramsd_invalid_alert,
    ET.SOFT_DISABLE: soft_disable_alert("paramsd 临时错误"),
  },
  EventName.paramsdPermanentError: {
    ET.NO_ENTRY: NoEntryAlert("paramsd 永久错误"),
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert("paramsd 永久错误"),
    ET.PERMANENT: NormalPermanentAlert("paramsd 永久错误"),
  },
  # ********** 影响控制状态转换的事件 **********
  EventName.pcmEnable: {
    ET.ENABLE: EngagementAlert(AudibleAlert.engage),
  },
  EventName.buttonEnable: {
    ET.ENABLE: EngagementAlert(AudibleAlert.engage),
  },
  EventName.pcmDisable: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
  },
  EventName.buttonCancel: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
    ET.NO_ENTRY: NoEntryAlert("已按取消"),
  },
  EventName.brakeHold: {
    ET.WARNING: Alert(
      "按恢复退出刹车保持",
      "",
      AlertStatus.userPrompt, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .2),
  },
  EventName.parkBrake: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
    ET.NO_ENTRY: NoEntryAlert("手刹已拉起"),
  },
  EventName.pedalPressed: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
    ET.NO_ENTRY: NoEntryAlert("已踩刹车",
                              visual_alert=VisualAlert.brakePressed),
  },
  EventName.steerDisengage: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
    ET.NO_ENTRY: NoEntryAlert("已转动方向盘"),
  },
  EventName.preEnableStandstill: {
    ET.PRE_ENABLE: Alert(
      "松开刹车以启用",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, .1, creation_delay=1.),
  },
  EventName.gasPressedOverride: {
    ET.OVERRIDE_LONGITUDINAL: Alert(
      "",
      "",
      AlertStatus.normal, AlertSize.none,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, .1),
  },
  EventName.steerOverride: {
    ET.OVERRIDE_LATERAL: Alert(
      "",
      "",
      AlertStatus.normal, AlertSize.none,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, .1),
  },
  EventName.wrongCarMode: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
    ET.NO_ENTRY: wrong_car_mode_alert,
  },
  EventName.resumeBlocked: {
    ET.NO_ENTRY: NoEntryAlert("按 SET 键启用"),
  },
  EventName.wrongCruiseMode: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
    ET.NO_ENTRY: NoEntryAlert("自适应巡航已关闭"),
  },
  EventName.steerTempUnavailable: {
    ET.SOFT_DISABLE: soft_disable_alert("转向暂时不可用"),
    ET.NO_ENTRY: NoEntryAlert("转向暂时不可用"),
  },
  EventName.steerTimeLimit: {
    ET.SOFT_DISABLE: soft_disable_alert("车辆转向时长限制"),
    ET.NO_ENTRY: NoEntryAlert("车辆转向时长限制"),
  },
  EventName.outOfSpace: {
    ET.PERMANENT: out_of_space_alert,
    ET.NO_ENTRY: NoEntryAlert("存储空间不足"),
  },
  EventName.belowEngageSpeed: {
    ET.NO_ENTRY: below_engage_speed_alert,
  },
  EventName.sensorDataInvalid: {
    ET.PERMANENT: Alert(
      "传感器数据无效",
      "可能为硬件问题",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOWER, VisualAlert.none, AudibleAlert.none, .2, creation_delay=1.),
    ET.NO_ENTRY: NoEntryAlert("传感器数据无效"),
    ET.SOFT_DISABLE: soft_disable_alert("传感器数据无效"),
  },
  EventName.noGps: {
  },
  EventName.tooDistracted: {
    ET.NO_ENTRY: NoEntryAlert("分心程度过高"),
  },
  EventName.excessiveActuation: {
    ET.SOFT_DISABLE: soft_disable_alert("执行器动作过大"),
    ET.NO_ENTRY: NoEntryAlert("执行器动作过大"),
  },
  EventName.overheat: {
    ET.PERMANENT: overheat_alert,
    ET.SOFT_DISABLE: soft_disable_alert("系统过热"),
    ET.NO_ENTRY: NoEntryAlert("系统过热"),
  },
  EventName.wrongGear: {
    ET.SOFT_DISABLE: user_soft_disable_alert("档位非 D"),
    ET.NO_ENTRY: NoEntryAlert("档位非 D"),
  },
  # 当校准角度超出可接受范围时触发。例如设备指向过左或过右。
  # 通常需完全移除支架并重新安装，确保设备正对前方且水平。
  # 详见 https://comma.ai/setup
  EventName.calibrationInvalid: {
    ET.PERMANENT: calibration_invalid_alert,
    ET.SOFT_DISABLE: soft_disable_alert("校准无效：重新安装设备并校准"),
    ET.NO_ENTRY: NoEntryAlert("校准无效：重新安装设备并校准"),
  },
  EventName.calibrationIncomplete: {
    ET.PERMANENT: calibration_incomplete_alert,
    ET.SOFT_DISABLE: soft_disable_alert("校准未完成"),
    ET.NO_ENTRY: NoEntryAlert("校准进行中"),
  },
  EventName.calibrationRecalibrating: {
    ET.PERMANENT: calibration_incomplete_alert,
    ET.SOFT_DISABLE: soft_disable_alert("检测到设备重新安装：正在重新校准"),
    ET.NO_ENTRY: NoEntryAlert("检测到重新安装：正在重新校准"),
  },
  EventName.doorOpen: {
    ET.SOFT_DISABLE: user_soft_disable_alert("车门未关"),
    ET.NO_ENTRY: NoEntryAlert("车门未关"),
  },
  EventName.seatbeltNotLatched: {
    ET.SOFT_DISABLE: user_soft_disable_alert("安全带未系"),
    ET.NO_ENTRY: NoEntryAlert("安全带未系"),
  },
  EventName.espDisabled: {
    ET.SOFT_DISABLE: soft_disable_alert("电子稳定控制已关闭"),
    ET.NO_ENTRY: NoEntryAlert("电子稳定控制已关闭"),
  },
  EventName.lowBattery: {
    ET.SOFT_DISABLE: soft_disable_alert("电量不足"),
    ET.NO_ENTRY: NoEntryAlert("电量不足"),
  },
  # openpilot 服务之间按一定间隔通信。若通信不规律则触发此警告。
  # 可能为服务崩溃、消息未按时发送或平均间隔过高。
  EventName.commIssue: {
    ET.SOFT_DISABLE: soft_disable_alert("进程间通信异常"),
    ET.NO_ENTRY: comm_issue_alert,
  },
  EventName.commIssueAvgFreq: {
    ET.SOFT_DISABLE: soft_disable_alert("进程间通信频率过低"),
    ET.NO_ENTRY: NoEntryAlert("进程间通信频率过低"),
  },
  EventName.selfdrivedLagging: {
    ET.SOFT_DISABLE: soft_disable_alert("系统延迟"),
    ET.NO_ENTRY: NoEntryAlert("Selfdrive 进程延迟：请重启设备"),
  },
  # 管理器检测到服务在行驶中意外退出
  EventName.processNotRunning: {
    ET.NO_ENTRY: process_not_running_alert,
    ET.SOFT_DISABLE: soft_disable_alert("进程未运行"),
  },
  EventName.radarFault: {
    ET.SOFT_DISABLE: soft_disable_alert("雷达错误：请重启车辆"),
    ET.NO_ENTRY: NoEntryAlert("雷达错误：请重启车辆"),
  },
  EventName.radarTempUnavailable: {
    ET.SOFT_DISABLE: soft_disable_alert("雷达暂时不可用"),
    ET.NO_ENTRY: NoEntryAlert("雷达暂时不可用"),
  },
  # 模型应处理每一帧。若处理过慢需丢帧。当丢帧超过20%时触发。
  EventName.modeldLagging: {
    ET.SOFT_DISABLE: soft_disable_alert("驾驶模型延迟"),
    ET.NO_ENTRY: NoEntryAlert("驾驶模型延迟"),
    ET.PERMANENT: modeld_lagging_alert,
  },
  # 模型预测当前车速和转速。若行驶中速度不确定性高，通常表示场景理解困难。
  EventName.posenetInvalid: {
    ET.SOFT_DISABLE: soft_disable_alert("Posenet 速度无效"),
    ET.NO_ENTRY: posenet_invalid_alert,
  },
  # 定位器检测到 >40 m/s² (~4G) 加速度，提醒设备可能从挡风玻璃脱落。
  EventName.deviceFalling: {
    ET.SOFT_DISABLE: soft_disable_alert("设备从支架脱落"),
    ET.NO_ENTRY: NoEntryAlert("设备从支架脱落"),
  },
  EventName.lowMemory: {
    ET.SOFT_DISABLE: soft_disable_alert("内存不足：请重启设备"),
    ET.PERMANENT: low_memory_alert,
    ET.NO_ENTRY: NoEntryAlert("内存不足：请重启设备"),
  },
  EventName.accFaulted: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert("定速巡航故障：请重启车辆"),
    ET.PERMANENT: NormalPermanentAlert("定速巡航故障：重启车辆后启用"),
    ET.NO_ENTRY: NoEntryAlert("定速巡航故障：请重启车辆"),
  },
  EventName.espActive: {
    ET.SOFT_DISABLE: soft_disable_alert("电子稳定控制激活"),
    ET.NO_ENTRY: NoEntryAlert("电子稳定控制激活"),
  },
  EventName.controlsMismatch: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert("控制信号不匹配"),
    ET.NO_ENTRY: NoEntryAlert("控制信号不匹配"),
  },
  # 设备 USB 栈可能进入错误状态，导致与 panda 断开连接
  EventName.usbError: {
    ET.SOFT_DISABLE: soft_disable_alert("USB 错误：请重启设备"),
    ET.PERMANENT: NormalPermanentAlert("USB 错误：请重启设备"),
    ET.NO_ENTRY: NoEntryAlert("USB 错误：请重启设备"),
  },
  # 可能原因：
  # - 完全无 CAN 数据
  # - 有 CAN 数据但部分消息频率不对
  # 若非开发新车接口，通常为线路问题
  EventName.canError: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert("CAN 错误"),
    ET.PERMANENT: Alert(
      "CAN 错误：请检查连接",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, 1., creation_delay=1.),
    ET.NO_ENTRY: NoEntryAlert("CAN 错误：请检查连接"),
  },
  EventName.canBusMissing: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert("CAN 总线断开"),
    ET.PERMANENT: Alert(
      "CAN 总线断开：疑似线缆故障",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, 1., creation_delay=1.),
    ET.NO_ENTRY: NoEntryAlert("CAN 总线断开：请检查连接"),
  },
  EventName.steerUnavailable: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert("LKAS 故障：请重启车辆"),
    ET.PERMANENT: NormalPermanentAlert("LKAS 故障：重启车辆后启用"),
    ET.NO_ENTRY: NoEntryAlert("LKAS 故障：请重启车辆"),
  },
  EventName.reverseGear: {
    ET.PERMANENT: Alert(
      "倒车\n档",
      "",
      AlertStatus.normal, AlertSize.full,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, .2, creation_delay=0.5),
    ET.USER_DISABLE: ImmediateDisableAlert("倒车档"),
    ET.NO_ENTRY: NoEntryAlert("倒车档"),
  },
  # 使用原厂 ACC 的车辆可能因各种原因取消 ACC。此时无法控制车辆，需立即警告。
  EventName.cruiseDisabled: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert("定速巡航已关闭"),
  },
  # 线束盒继电器打开时，LKAS 摄像头与车辆其余部分 CAN 总线分离。
  # 若在车辆侧收到 LKAS 摄像头消息，通常表示继电器未正确打开。
  EventName.relayMalfunction: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert("线束继电器故障"),
    ET.PERMANENT: NormalPermanentAlert("线束继电器故障", "请检查硬件"),
    ET.NO_ENTRY: NoEntryAlert("线束继电器故障"),
  },
  EventName.speedTooLow: {
    ET.IMMEDIATE_DISABLE: Alert(
      "openpilot 已取消",
      "速度过低",
      AlertStatus.normal, AlertSize.mid,
      Priority.HIGH, VisualAlert.none, AudibleAlert.disengage, 3.),
  },
  # 车速超过训练数据中大多数车辆时，模型输出可能不可预测。
  EventName.speedTooHigh: {
    ET.WARNING: Alert(
      "速度过高",
      "模型在此速度下不确定",
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.HIGH, VisualAlert.steerRequired, AudibleAlert.promptRepeat, 4.),
    ET.NO_ENTRY: NoEntryAlert("减速后启用"),
  },
  EventName.vehicleSensorsInvalid: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert("车辆传感器无效"),
    ET.PERMANENT: NormalPermanentAlert("车辆传感器校准中", "请行驶以校准"),
    ET.NO_ENTRY: NoEntryAlert("车辆传感器校准中"),
  },
  EventName.personalityChanged: {
    ET.WARNING: personality_changed_alert,
  },
  EventName.userBookmark: {
    ET.PERMANENT: NormalPermanentAlert("书签已保存", duration=1.5),
  },
  EventName.audioFeedback: {
    ET.PERMANENT: audio_feedback_alert,
  },
}
if __name__ == '__main__':
  # 按类型和优先级打印所有警告
  from cereal.services import SERVICE_LIST
  from collections import defaultdict
  event_names = {v: k for k, v in EventName.schema.enumerants.items()}
  alerts_by_type: dict[str, dict[Priority, list[str]]] = defaultdict(lambda: defaultdict(list))
  CP = car.CarParams.new_message()
  CS = car.CarState.new_message()
  sm = messaging.SubMaster(list(SERVICE_LIST.keys()))
  for i, alerts in EVENTS.items():
    for et, alert in alerts.items():
      if callable(alert):
        alert = alert(CP, CS, sm, False, 1, log.LongitudinalPersonality.standard)
      alerts_by_type[et][alert.priority].append(event_names[i])
  all_alerts: dict[str, list[tuple[Priority, list[str]]]] = {}
  for et, priority_alerts in alerts_by_type.items():
    all_alerts[et] = sorted(priority_alerts.items(), key=lambda x: x[0], reverse=True)
  for status, evs in sorted(all_alerts.items(), key=lambda x: x[0]):
    print(f"**** {status} ****")
    for p, alert_list in evs:
      print(f" {repr(p)}:")
      print(" ", ', '.join(alert_list), "\n")