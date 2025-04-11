import paho.mqtt.client as mqtt
import io
import base64
import json
from stt import transcribe_stt
from search_chromadb import search_hospital_info
from prompting import generate_response, clear_history
import threading
from search_mysql import get_image_for_keyword
from hospital_google_search import google_search
from tavily_search import optimize_query

class MQTTChatbot:
    def __init__(self):
        self.BROKER = "j12e103.p.ssafy.io"
        self.PORT = 1883

        self.TOPIC_SUBSCRIBE = "user/chatbot/request"
        self.TOPIC_PUBLISH = "chatbot/response"

        self.TIMEOUT_SECONDS = 60 * 3  # 3분 후 history 초기화
        self.history_reset_timer = None  # 타이머 변수

        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, protocol=mqtt.MQTTv311)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

    # MQTT 연결
    def start(self):
        try:
            self.client.connect(self.BROKER, self.PORT, 60)
            print("🔄 MQTT 브로커에 연결 중...")
            self.client.loop_forever()
        except Exception as e:
            print(f"❌ MQTT 브로커 연결 오류: {e}")

    # MQTT 연결 이벤트
    # MQTT 구독
    def on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code == 0:
            print("✅ MQTT 브로커 연결 성공")
            client.subscribe(self.TOPIC_SUBSCRIBE)
        else:
            print(f"❌ MQTT 브로커 연결 실패 (코드: {reason_code})")

    def on_message(self, client, userdata, msg):
        """MQTT 메시지 수신 이벤트"""

        try:
            # 메시지 디코딩 및 JSON 파싱
            message = json.loads(msg.payload.decode('utf-8'))
            user_id = message.get("user_id", "")
            raw_payload = message.get("payload", "")

            if not user_id:
                print("⚠️ user_id가 없습니다.")
                return

            # 기존 타이머 취소 (유저별 타이머가 필요한 경우에는 dict로 관리 필요)
            if self.history_reset_timer:
                self.history_reset_timer.cancel()

            print(f"📩 수신한 메시지 from user_id: {user_id}, 크기: {len(raw_payload)} bytes")

            # 텍스트 or 오디오 분기
            if len(raw_payload) < 1024:
                transcribed_text = raw_payload
                print("📄 텍스트 데이터 수신:", transcribed_text)
            else:
                transcribed_text = self.process_audio(raw_payload.encode('utf-8'))

            if not transcribed_text:
                return

            # 내부 검색
            search_results = search_hospital_info(transcribed_text)

            #외부 검색
            external_search= []

            # 최적화된 검색어 생ㅇ성
            optimized_query = optimize_query(search_results)
            print("최적화된 검색어: ", optimized_query)

            # Google 검색 결과 추가
            google_results = google_search("site:samsunghospital.com " , optimized_query)
            if google_results:
                print("Google 검색 결과 추가")
                external_search.extend(google_results)  # 기존 검색 결과에 추가
                print("🔎 Google 검색 결과를 추가했습니다.")

            response_text = generate_response(transcribed_text, search_results, external_search)

            facility_name = ""
            image_url = ""
            
            keywords = ["간호사실", "501호실", "502호실", "503호실"]
            
            if "5층" in response_text:
                found_facilities = []  # 5층에 있는 시설명을 저장할 리스트
                
                for result in search_results:
                    if result.get("floor_info") == "5층":
                        found_facilities.append(result.get("facility_name", ""))

                print(f"🏢 [DEBUG] 5층에서 찾은 시설들: {found_facilities}")

                # 5층 시설 중에서 우리가 찾는 키워드가 포함된 시설명을 우선 선택
                facility_name = next((f for f in found_facilities if f in keywords), "")

                if facility_name:
                    print(f"✅ [DEBUG] 키워드 매칭된 facility_name: {facility_name}")
                    image_url = get_image_for_keyword(facility_name)  # 이미지 조회
                    print(f"📸 조회된 이미지 URL: {image_url}")
                else:
                    print(f"⚠️ [DEBUG] 키워드 매칭 실패. 기본 facility_name 사용.")
                    facility_name = found_facilities[0] if found_facilities else ""  # 첫 번째 시설 선택

                response_text += " 안내를 시작할까요?"

            # 최종 응답 생성
            message_data = {
                "user_id": user_id,
                "request": transcribed_text,
                "response": response_text,
                "facility": facility_name,
                "image": image_url
            }

            topic_response = f"chatbot/{user_id}/response"
            message_json = json.dumps(message_data, ensure_ascii=False)
            client.publish(topic_response, message_json)
            print(f"✅ 응답이 {topic_response} 토픽으로 발행되었습니다.")

            # 타이머 재시작
            self.history_reset_timer = threading.Timer(self.TIMEOUT_SECONDS, self.reset_history)
            self.history_reset_timer.start()

        except Exception as e:
            print(f"⚠️ 메시지 처리 중 오류 발생: {e}")

    def process_audio(self, audio_payload):
        """바이너리 음성 데이터를 텍스트로 변환"""
        try:
            audio_data = audio_payload.decode('utf-8')

            # Base64 패딩 보정
            padding = '=' * (4 - len(audio_data) % 4)
            audio_data += padding

            # Base64 디코딩
            audio_data = base64.b64decode(audio_data)
            audio_buffer = io.BytesIO(audio_data)

            print("음성 파일 처리 중...")
            return transcribe_stt(audio_buffer)

        except Exception as e:
            print(f"❌ 음성 처리 중 오류 발생: {e}")
            return None

    def reset_history(self):
        """대화 기록 초기화"""
        print("⏳ 대화 기록 초기화 (사용자 응답 없음)")
        clear_history()
        self.history_reset_timer = None


if __name__ == "__main__":
    chatbot = MQTTChatbot()
    chatbot.start()
