export interface ChatRequest {
    message: string;
    model?: string;
}

export interface ChatResponse {
    reply: string;
    model_name: string;
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

export interface ChatMsg {
    role: 'system' | 'user'
    content: string
    model?: string
}
