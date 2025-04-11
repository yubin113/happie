// hooks/useChatbotResponse.ts
import { useCallback } from "react";
import { mqttClient, mqttClientId } from "@/lib/mqttClient";

export const sendMessage = (data: string) => {
  const message = JSON.stringify({
    user_id: mqttClientId, // 여기서 clientId 넣어줌!
    payload: data,
  });
  mqttClient.publish("user/chatbot/request", message);
  console.log("📤 MQTT 메시지 전송:", message);
};


type ChatbotResponseProps = {
  setQuestion: (q: string) => void;
  setAnswer: (a: string) => void;
  setStage: (s: "idle" | "recording" | "loading" | "answering") => void;
  setShowWarning: (show: boolean) => void;
  setFacility?: (f: string | null) => void;
  setNavigationImage?: (url: string | null) => void;
};

export function useChatbotResponse({
  setQuestion,
  setAnswer,
  setStage,
  setShowWarning,
  setFacility,
  setNavigationImage,
}: ChatbotResponseProps) {
  const handleChatResponse = useCallback(
    (topic: string, message: Uint8Array | string) => {
      const msg = message.toString().trim();
      console.log("📩 수신된 메시지 원본:", msg);

      if (topic.startsWith("chatbot/") && topic.endsWith("/response")) {
        const parsed = JSON.parse(msg);

        // 👉 내 clientId랑 다르면 무시
        if (parsed.user_id !== mqttClientId) return;
        
        try {
          const parsed = JSON.parse(msg);
          setQuestion(parsed.request || "");
          setAnswer(parsed.response || msg);
          if (parsed.facility) setFacility?.(parsed.facility);
          if (parsed.image) setNavigationImage?.(parsed.image); // ✅ 여기서 저장
        } catch (e) {
          console.warn("❌ 파싱 실패:", e);
          setQuestion("");
          setAnswer(msg);
        }
        setStage("answering");
      }

      if (topic === "fall_detection") {
        setShowWarning(true);
      }
    },
    [setQuestion, setAnswer, setStage, setShowWarning, setFacility, setNavigationImage]
  );

  return { handleChatResponse };
}
