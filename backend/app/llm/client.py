from langchain_openai import ChatOpenAI
from langchain_community.chat_models import ChatOllama
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.output_parsers import StrOutputParser
from app.config import settings
import asyncio # 引入 asyncio 处理超时

def get_llm_model(model_name: str):
    """
    根据模型名称返回对应的 LangChain 模型实例
    """
    model_name = model_name.lower()

    if "/" in model_name and not "all-minilm" in model_name:
        print(f"🚀 Using SiliconFlow API: {model_name}")
        return ChatOpenAI(
            model=model_name,
            api_key=settings.SILICONFLOW_API_KEY,
            base_url=settings.SILICONFLOW_BASE_URL,
            temperature=0.7,
            timeout=10.0 # 硅基流动速度很快，10秒足够
        )
    

def ask_ai(text: str, system_prompt: str, model_name: str = None) -> tuple[str, str]:
    """
    通用 AI 调用函数
    Returns: (ai_reply_text, used_model_name)
    """
    # 1. 确定要使用的模型名称
    target_model = model_name if model_name else settings.DEFAULT_LLM_MODEL
    
    try:
        # 2. 获取 LangChain 模型实例
        llm = get_llm_model(target_model)
        print(f"🤖 Invoking model: {target_model}")
        
        # 3. 构建 Prompt
        prompt = ChatPromptTemplate.from_messages([
            ("system", system_prompt),
            ("user", "{input}")
        ])

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
    异步验证模型连接，防止阻塞主线程。
    """
    try:
        # 1. 获取模型实例
        llm = get_llm_model(model_name)
        print(f"🔍 Validating model connection: {model_name}")
        print(f"LLM Details: {llm}")
        # 2. [关键修改] 使用 ainvoke (异步) 代替 invoke (同步)
        # 并增加 timeout 控制，防止一直卡着
        try:
            # 设定 5 秒超时，连不上就立刻返回失败，不要傻等
            response = await asyncio.wait_for(llm.ainvoke("hi"), timeout=5.0)
        except asyncio.TimeoutError:
            return False, "Connection Timeout (5s)"

        # 3. 检查结果
        if response and response.content:
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