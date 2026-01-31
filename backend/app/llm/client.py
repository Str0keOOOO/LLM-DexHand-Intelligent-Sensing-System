import os
import asyncio
import json
from langchain_openai import ChatOpenAI
from langchain_community.chat_models import ChatOllama
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.output_parsers import StrOutputParser


AVAILABLE_MODELS = os.getenv("AVAILABLE_MODELS")
AVAILABLE_MODELS = json.loads(AVAILABLE_MODELS)


def get_llm_model(model_name: str):
    """
    根据模型名称返回对应的 LangChain 模型实例
    """
    model_name = model_name.lower() if model_name else ""

    if model_name == "qwen/qwen3-8b":
        return ChatOpenAI(
            model=model_name,
            api_key=os.getenv("SILICONFLOW_API_KEY"),
            base_url=os.getenv("SILICONFLOW_BASE_URL"),
            temperature=0.7,
            timeout=60.0,
            max_retries=2,
        )


async def ask_ai(text: str, system_prompt: str, model_name: str = None) -> tuple[str, str]:
    """
    通用 AI 调用函数 (异步版本)
    Returns: (ai_reply_text, used_model_name)
    """
    if not model_name:
        if AVAILABLE_MODELS:
            default_model = AVAILABLE_MODELS[0]["value"]
    else:
        default_model = model_name

    target_model = default_model

    try:
        llm = get_llm_model(target_model)
        prompt = ChatPromptTemplate.from_messages([("system", system_prompt), ("user", "{input}")])
        chain = prompt | llm | StrOutputParser()
        
        # 关键修改：使用 await 和 ainvoke
        response_text = await chain.ainvoke({"input": text})

        return response_text, target_model

    except Exception as e:
        return f"模型 {target_model} 调用失败: {str(e)}", target_model


async def validate_model(model_name: str) -> tuple[bool, str]:
    """
    异步验证模型连接
    """
    try:
        llm = get_llm_model(model_name)
        chain = llm | StrOutputParser()
        response = await asyncio.wait_for(chain.ainvoke("hi"), timeout=5.0)
        if response:
            return True, "Connected successfully"
        return False, "Empty response from model"

    except asyncio.TimeoutError:
        return False, "Connection Timeout (5s)"
    except ValueError as e:
        return False, str(e)
    except Exception as e:
        error_msg = str(e)
        print(f"❌ Validation Error: {error_msg}")
        if "401" in error_msg:
            return False, "Auth Failed (Check API Key)"
        if "Connection refused" in error_msg:
            return False, "Connection Refused (Is Ollama running?)"
        return False, f"Error: {error_msg[:100]}..."
