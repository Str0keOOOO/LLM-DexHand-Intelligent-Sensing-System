import os
import asyncio
from langchain_openai import ChatOpenAI
from langchain_community.chat_models import ChatOllama
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.output_parsers import StrOutputParser
import json


AVAILABLE_MODELS = os.getenv("AVAILABLE_MODELS")
print(f"🔍 Loaded AVAILABLE_MODELS env: {AVAILABLE_MODELS}")
AVAILABLE_MODELS = json.loads(AVAILABLE_MODELS)


def get_llm_model(model_name: str):
    """
    根据模型名称返回对应的 LangChain 模型实例
    """
    model_name = model_name.lower() if model_name else ""

    # --- 情况 A: 硅基流动 / DeepSeek (OpenAI 协议) ---
    if "/" in model_name or "gpt" in model_name:
        print(f"🚀 Using SiliconFlow/OpenAI API: {model_name}")
        return ChatOpenAI(
            model=model_name,
            api_key=os.getenv("SILICONFLOW_API_KEY") or os.getenv("OPENAI_API_KEY"),
            base_url=os.getenv("SILICONFLOW_BASE_URL", "https://api.siliconflow.cn/v1"),
            temperature=0.7,
            timeout=10.0,
            max_retries=2,
        )

    # --- 情况 B: 本地 Ollama ---
    else:
        print(f"🦙 Using Local Ollama: {model_name or 'default'}")
        return ChatOllama(model=model_name if model_name else "llama3", base_url=os.getenv("OLLAMA_BASE_URL", "http://localhost:11434"), temperature=0.7, timeout=30.0)


def ask_ai(text: str, system_prompt: str, model_name: str = None) -> tuple[str, str]:
    """
    通用 AI 调用函数
    Returns: (ai_reply_text, used_model_name)
    """
    # [关键修改] 1. 确定要使用的模型名称
    # 如果没传 model_name，则默认使用列表中的第一个模型
    if not model_name:
        if AVAILABLE_MODELS:
            default_model = AVAILABLE_MODELS[0]["value"]
        else:
            default_model = "gpt-3.5-turbo"  # 最后的保底
    else:
        default_model = model_name

    target_model = default_model

    try:
        # 2. 获取 LangChain 模型实例
        llm = get_llm_model(target_model)
        print(f"🤖 Invoking model: {target_model}")

        # 3. 构建 Prompt
        prompt = ChatPromptTemplate.from_messages([("system", system_prompt), ("user", "{input}")])

        # 4. 构建链 (Chain)
        chain = prompt | llm | StrOutputParser()

        # 5. 执行
        response_text = chain.invoke({"input": text})

        return response_text, target_model

    except Exception as e:
        print(f"❌ LLM Error ({target_model}): {e}")
        return f"模型 {target_model} 调用失败: {str(e)}", target_model


async def validate_model(model_name: str) -> tuple[bool, str]:
    """
    异步验证模型连接
    """
    try:
        llm = get_llm_model(model_name)
        print(f"🔍 Validating model connection: {model_name}")

        try:
            chain = llm | StrOutputParser()
            # 发送简单的 hi 测试连接
            response = await asyncio.wait_for(chain.ainvoke("hi"), timeout=5.0)
        except asyncio.TimeoutError:
            return False, "Connection Timeout (5s)"

        if response:
            return True, "Connected successfully"
        else:
            return False, "Empty response from model"

    except Exception as e:
        error_msg = str(e)
        print(f"❌ Validation Error: {error_msg}")
        if "401" in error_msg:
            return False, "Auth Failed (Check API Key)"
        if "Connection refused" in error_msg:
            return False, "Connection Refused (Is Ollama running?)"
        return False, f"Error: {error_msg[:100]}..."
