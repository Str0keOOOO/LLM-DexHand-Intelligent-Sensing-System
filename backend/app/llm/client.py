from openai import OpenAI
from app.config import settings

client = OpenAI(api_key=settings.LLM_API_KEY, base_url=settings.LLM_BASE_URL)


def ask_ai(text: str, system_prompt: str) -> str:
    """纯粹的 AI 调用函数"""
    try:
        resp = client.chat.completions.create(model=settings.LLM_MODEL_NAME, messages=[{"role": "system", "content": system_prompt}, {"role": "user", "content": text}])
        return resp.choices[0].message.content
    except Exception as e:
        print(f"LLM Error: {e}")
        return "思考失败"
