"use client";

import { useEffect, useState, useCallback } from "react";
import Warning from "./components/Warning";
import QuestionButton from "./components/QuestionButton";
import VoiceButton from "./components/VoiceButton";
import DotAnimation from "./components/DotAnimation";
import EyeTracker from "./components/EyeTracker";
import { mqttClient } from "@/lib/mqttClient";
import { useChatbotResponse } from "./hooks/useChatbotResponse";
import Swal from "sweetalert2";
import { colorOptions } from "../../types/color";
import "./../globals.css";

type Stage = "idle" | "recording" | "loading" | "answering" | "navigating";

export default function BotLayout() {
  const [stage, setStage] = useState<Stage>("idle");
  const [question, setQuestion] = useState("");
  const [answer, setAnswer] = useState("");
  const [typedAnswer, setTypedAnswer] = useState("");
  const [isTypingDone, setIsTypingDone] = useState(false);
  const [facility, setFacility] = useState<string | null>(null);
  const [showWarning, setShowWarning] = useState(false);
  const [selectedQuestion, setSelectedQuestion] = useState<string | null>(null);
  const [navigationImage, setNavigationImage] = useState<string | null>(null);
  const [navigationDone, setNavigationDone] = useState(false); // ✅ 안내 종료 메시지 제어용
  const [showInteraction, setShowInteraction] = useState(false);

  const questionList = ["원무수납처 \n어디야?", "소아진정실은 \n뭐하는 곳이야?", "501호실이 \n어디있어?"].map((text, idx) => ({
    text,
    color: colorOptions[idx % colorOptions.length],
  }));

  const { handleChatResponse } = useChatbotResponse({
    setQuestion,
    setAnswer,
    setStage,
    setShowWarning,
    setFacility,
    setNavigationImage, // ✅ 수신된 이미지 기억해두기
  });

  const onMqttMessage = useCallback(
    (topic: string, message: Buffer) => {
      // ✅ 안내 완료 메시지 수신 시 처리
      if (topic === "robot/log") {
        setNavigationDone(true); // 종료 메시지 보여주기

        // ✅ 3초 후 홈 화면으로 자동 전환
        setTimeout(() => {
          setStage("idle");
          setAnswer("");
          setTypedAnswer("");
          setIsTypingDone(false);
          setNavigationImage(null);
          setFacility(null);
          setNavigationDone(false); // 다시 숨기기
        }, 3000);
        return;
      }

      handleChatResponse(topic, message);
    },
    [handleChatResponse]
  );

  useEffect(() => {
    mqttClient.on("message", onMqttMessage);
    return () => {
      mqttClient.removeListener("message", onMqttMessage);
    };
  }, [onMqttMessage]);

  useEffect(() => {
    if (stage === "loading" || stage === "recording") {
      setAnswer("");
      setTypedAnswer("");
      setIsTypingDone(false);
    }
  }, [stage]);

  useEffect(() => {
    if (stage === "answering" && answer) {
      setTypedAnswer("");
      setIsTypingDone(false);

      window.speechSynthesis.cancel();
      const utter = new SpeechSynthesisUtterance(answer);
      utter.lang = "ko-KR";
      window.speechSynthesis.speak(utter);

      const steps = Array.from({ length: answer.length }, (_, i) => answer.slice(0, i + 1));
      let idx = 0;

      const interval = setInterval(() => {
        setTypedAnswer(steps[idx]);
        idx++;
        if (idx >= steps.length) {
          clearInterval(interval);
          setIsTypingDone(true);
        }
      }, 150);

      return () => clearInterval(interval);
    }
  }, [answer, stage]);

  return (
    <div className="flex flex-col items-center justify-center min-h-screen bg-gray-100 relative">
      {stage !== "navigating" && <EyeTracker />}

      {stage === "idle" && (
  <>
{!showInteraction && (
  <div className="flex flex-col items-center m-5">
    <button
      className="px-20 py-10 bg-emerald-500 hover:bg-emerald-600 text-white text-6xl rounded-xl shadow-md transition"
      onClick={() => setShowInteraction(true)}
    >
      🤖 하피가 도와드릴까요?
    <p className="mt-8 text-gray-100 text-2xl animate-bounce">▲ 여기를 눌러주세요</p>
    </button>
  </div>
)}

    {/* ✅ X 버튼 – 화면 우측 상단에 고정 */}
    {showInteraction && (
      <>
        <button
          onClick={() => setShowInteraction(false)}
          className="fixed top-6 right-6 z-50 text-3xl text-gray-400 hover:text-gray-600 transition"
          title="닫기"
        >
          &times;
        </button>

        {/* 질문 + 음성 버튼 묶음 */}
        <div className="flex flex-col items-center gap-6 w-full max-w-4xl">
          <div className="grid grid-cols-1 sm:grid-cols-2 md:grid-cols-3 gap-4 w-full">
            {questionList.map(({ text, color }, idx) => (
              <QuestionButton
                key={idx}
                text={text}
                color={color}
                selected={selectedQuestion === text}
                onSelect={() => setSelectedQuestion(text)}
                setQuestion={setQuestion}
                setAnswer={setAnswer}
                setStage={setStage}
              />
            ))}
          </div>

          <VoiceButton
            setQuestion={setQuestion}
            setAnswer={setAnswer}
            setStage={setStage}
            stage={stage}
            size={24}
          />
        </div>
      </>
    )}
  </>
)}



      {/* ✅ 녹음 중 UI */}
      {stage === "recording" && (
        <>
          <div className="absolute top-[calc(50%-7.5rem)] left-[calc(50%+20rem)] z-20 animate-slideInFromRight">
            <img src="/images/cat.gif" alt="귀 기울이는 중" className="w-[350px] h-[350px] animate-scalePulse" />
          </div>
          {/* 왼쪽 고양이 - transform 분리 */}
          <div className="absolute top-[calc(50%-7.5rem)] right-[calc(50%+20rem)] z-20 animate-slideInFromLeft">
            <div className="transform scale-x-[-1]">
              <img src="/images/cat.gif" alt="귀 기울이는 중" className="w-[350px] h-[350px] animate-scalePulse" />
            </div>
          </div>
          <div className="flex flex-col items-center justify-center text-center bg-white w-[400px] h-[100px] p-4 rounded-xl shadow-md">
            <p className="text-3xl text-gray-700 flex items-center">
              하피가 귀 기울이고 있어요 <DotAnimation />
            </p>
          </div>
        </>
      )}

      {/* ✅ 응답 로딩 중 UI */}
      {stage === "loading" && (
        <div className="flex items-center bg-white w-[400px] h-[100px] p-4 rounded-xl shadow-md">
          <img src="/images/voice-loading1.gif" alt="로딩 중" className="w-16 h-16 mr-4 ml-6" />
          <p className="text-3xl text-gray-800 flex items-center">
            흠.. 그게 말이죠 <DotAnimation />
          </p>
        </div>
      )}

      {/* ✅ 답변 출력 UI */}
      {stage === "answering" && (
        <div className="w-full max-w-5xl relative">
          <div className="bg-white p-6 rounded-xl shadow mb-3">
            <p className="text-2xl text-gray-500 mb-4">❓ 질문</p>
            <p className="text-3xl">{question}</p>
          </div>
          <div className="bg-blue-50 p-6 rounded-xl shadow">
            <p className="text-2xl text-blue-500 mb-4">🤖 하피의 답변</p>
            <p className="text-3xl whitespace-pre-wrap">
              {typedAnswer || (
                <div className="text-3xl wavy-text flex gap-0.5">
                  {"하피가 응답 중이에요".split("").map((char, idx) => (
                    <span key={idx}>{char}</span>
                  ))}
                  <DotAnimation />
                </div>
              )}
            </p>

            {/* ✅ 안내 시작 여부 확인 */}
            {isTypingDone && answer?.endsWith("안내를 시작할까요?") && facility && (
              <div className="flex gap-5 mt-4 justify-end items-center">
                <button
                  onClick={async () => {
                    window.speechSynthesis.cancel();
                    const result = await Swal.fire({
                      title: "안내를 시작할까요?",
                      text: `${facility}로 안내를 시작합니다.`,
                      icon: "question",
                      showCancelButton: true,
                      confirmButtonText: "네, 시작할게요!",
                      cancelButtonText: "취소",
                      reverseButtons: false,
                    });

                    if (result.isConfirmed) {
                      // 안내 명령 API 전송
                      try {
                        const res = await fetch("https://j12e103.p.ssafy.io/api/equipment/create-order", {
                          method: "POST",
                          headers: {
                            "Content-Type": "application/json",
                          },
                          body: JSON.stringify({
                            robot: "robot1",
                            place: facility,
                            todo: "안내",
                          }),
                        });

                        if (!res.ok) throw new Error("안내 명령 전송 실패");

                        console.log("✅ 안내 명령 전송 완료");
                      } catch (error) {
                        console.error("❌ 안내 명령 실패:", error);
                        Swal.fire({
                          icon: "error",
                          title: "안내 명령 실패",
                          text: "서버 전송 중 문제가 발생했어요!",
                        });
                        return;
                      }

                      setStage("navigating");
                    }
                  }}
                  className="w-20 h-10 bg-green-300 text-xl hover:bg-green-400 hover:scale-110 rounded-xl text-gray-800 shadow-md transition-all"
                >
                  예
                </button>
                <button
                  onClick={() => {
                    window.speechSynthesis.cancel();
                    setStage("idle");
                    setAnswer("");
                    setTypedAnswer("");
                    setIsTypingDone(false);
                  }}
                  className="w-20 h-10 bg-red-300 text-xl hover:bg-red-400 hover:scale-110 rounded-xl text-gray-800 shadow-md transition-all"
                >
                  아니오
                </button>
              </div>
            )}

            {/* ✅ 다시 질문하기 */}
            {isTypingDone && !answer?.endsWith("안내를 시작할까요?") && (
              <div className="pt-3 flex justify-end items-center gap-6">
                <VoiceButton setQuestion={setQuestion} setAnswer={setAnswer} setStage={setStage} stage={stage} size={14} />
                <button
                  onClick={() => {
                    window.speechSynthesis.cancel();
                    setStage("idle");
                    setAnswer("");
                    setTypedAnswer("");
                    setIsTypingDone(false);
                  }}
                  className="w-20 h-10 bg-indigo-500 text-xl text-white hover:bg-indigo-700 hover:scale-110 rounded-xl shadow-md transition-all"
                >
                  홈으로
                </button>
              </div>
            )}
          </div>
        </div>
      )}

      {stage === "navigating" && navigationImage && (
        <div className="flex flex-col items-center justify-center bg-white w-full h-full flex-grow gap-10">
          {!navigationDone && (
            <>
              <img src={navigationImage} alt="안내 중" className="rounded-xl w-full max-h-[600px] object-contain mb-4" />
              <p className="text-xl text-gray-800 font-semibold flex items-center">
                <div className="text-7xl text-emerald-700 wavy-text flex gap-2">
                  {"하피를 따라오세요!".split("").map((char, idx) => (
                    <span key={idx}>{char}</span>
                  ))}
                </div>
              </p>
            </>
          )}

          {/* ✅ 안내 종료 메시지만 출력 */}
          {navigationDone && <p className="text-7xl text-gray-700 animate-fadeIn transition-opacity duration-500">안내가 종료되었습니다.</p>}
        </div>
      )}

      {showWarning && <Warning onClose={() => setShowWarning(false)} />}
    </div>
  );
}
