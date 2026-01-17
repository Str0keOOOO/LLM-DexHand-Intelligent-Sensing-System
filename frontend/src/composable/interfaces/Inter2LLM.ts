// 找到 ChatRequest 和 ChatResponse 并修改为：

export interface ChatRequest {
    message: string;
    model?: string; // [新增] 可选的模型名称，例如 "deepseek-chat"
}

export interface ChatResponse {
    reply: string;
    model_name: string; // [新增] 后端实际使用的模型名字
    action_code?: string | null;
}

export interface CheckModelRequest {
    model: string;
}

export interface CheckModelResponse {
    success: boolean;
    message: string;
}

export interface ModelOption {
    label: string;
    value: string;
}

export interface ModelListResponse {
    models: ModelOption[];
}