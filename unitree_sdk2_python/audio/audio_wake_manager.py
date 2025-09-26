import time
import re
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

# 可编辑的唤醒词（客户端层面）——把你想要的唤醒词写到这里
WAKE_WORDS = [
    r"小羲",         # 精确词
    r"你好小羲",
    r"哈喽",       # 示例
    r"唤醒"          # 你可以加正则
]

CONFIDENCE_THRESHOLD = 0.20  # 置信度阈值，按需调高
SESSION_TIMEOUT = 5.0        # 唤醒后等待用户发言的秒数，超时回睡

class WakeDialogManager:
    def __init__(self, audio_client: AudioClient):
        self.audio_client = audio_client
        self.awake = False
        self.last_heard_time = 0.0

    def _matches_wake(self, text: str, confidence: float):
        if not text:
            return False
        if confidence < CONFIDENCE_THRESHOLD:
            return False
        for pattern in WAKE_WORDS:
            if re.search(pattern, text, re.IGNORECASE):
                return True
        return False

    def on_asr_message(self, msg_obj):
        """
        msg_obj is expected to be a dict matching the ASR JSON:
        {
           "index": ...,
           "text": "...",
           "angle": ...,
           "speaker_id": ...,
           "confidence": 0.95,
           "language": "zh-CN",
           "is_final": true
        }
        """
        try:
            data = msg_obj  # if you receive raw String_, parse its .data() into dict externally
            text = data.get("text", "") or ""
            confidence = float(data.get("confidence", 0.0))
            is_final = data.get("is_final", True)

            # only act on final segments (default ASR is non-streaming but keep the guard)
            if not is_final:
                return

            now = time.time()
            if self._matches_wake(text, confidence):
                # 进入唤醒态
                self.awake = True
                self.last_heard_time = now
                # 反馈 LED + TTS
                self.audio_client.LedControl(0, 255, 0)   # 绿色表示唤醒
                self.audio_client.TtsMaker("我在，请说。", 0)
                print("[WAKE] Wake word detected:", text, confidence)
                return

            # 若已处于唤醒态，解释/处理对话
            if self.awake:
                self.last_heard_time = now
                # 简单意图匹配示例
                if "时间" in text or "几点" in text:
                    self.audio_client.TtsMaker("现在是北京时间 " + time.strftime("%H点%M分"), 0)
                elif "你的名字" in text or "你是谁" in text:
                    self.audio_client.TtsMaker("我是金羲科技的机器人。", 0)
                elif "再见" in text or "退出" in text:
                    self.audio_client.TtsMaker("再见。", 0)
                    self._sleep()
                else:
                    # 默认回声（也可以把 text 发到外部 NLU/LLM）
                    self.audio_client.TtsMaker("你说的是：" + text, 0)

        except Exception as e:
            print("[ERROR] on_asr_message:", e)

    def tick(self):
        """定期调用，检查会话超时"""
        if self.awake and (time.time() - self.last_heard_time) > SESSION_TIMEOUT:
            self._sleep()

    def _sleep(self):
        self.awake = False
        self.audio_client.LedControl(0, 0, 0)  # 关闭灯
        print("[SLEEP] session timeout; go to sleep.")

def audio_msg_handler(msg):
    try:
        import json
        data = json.loads(msg.data()) if hasattr(msg, "data") else msg
    except Exception:
        data = {}
    mgr.on_asr_message(data)

if __name__ == "__main__":
    
    # 修改网卡名称
    ChannelFactoryInitialize(0, "enp60s0") 

    audio_client = AudioClient()
    audio_client.Init()
    audio_client.SetTimeout(10.0)

    mgr = WakeDialogManager(audio_client)

    sub = ChannelSubscriber("rt/audio_msg", String_)
    sub.Init(audio_msg_handler, 10)

    try:
        while True:
            mgr.tick()
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
