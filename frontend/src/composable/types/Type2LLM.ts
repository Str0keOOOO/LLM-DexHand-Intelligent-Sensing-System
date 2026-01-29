export type ModelVal = string | { value: string } | null | undefined

export type ConnStatus = 'init' | 'checking' | 'success' | 'fail'
//  TODO 修改类型
export type ConnStatus1 = 'checking' | 'success' | 'fail' | string

export type ModelOption = { label: string; value: string }

export type ChatMsg = {
    role: 'user' | 'system' | string
    content: string
    model?: string
}
export type ControlRespPayload = {
    sent_to?: string
    sentTo?: string
    result?: { sent_to?: string }
    message?: string
    error?: string
}