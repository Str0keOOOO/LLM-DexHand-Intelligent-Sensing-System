import request from '@/composable/utils/request'
import type { RosHealth } from "@/composable/interfaces/Inter2Robot.ts";

export function buildUrl(path?: string) {
    const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
    const p = path ?? '/api/ros_ws/robot-data';
    const normalized = p.startsWith('/') ? p : '/' + p;
    return `${protocol}//${location.host}${normalized}`;
}

export function safeParse(raw: unknown) {
    if (typeof raw !== 'string') return null;
    try {
        return JSON.parse(raw);
    } catch (e) {
        console.error('WebSocket message parse error:', e);
        return null;
    }
}

export async function rosHealth() {
    return request.get<RosHealth>('/ros_ws/health')
}

// --- 新增：手动控制相关 ---

export interface ControlCommand {
    hand: 'left' | 'right';
    joints: Record<string, number>;
}

export async function sendControlCommand(data: ControlCommand) {
    // 调用后端新增的 POST /control 接口
    // 注意：根据之前的习惯，路由前缀可能是 /ros_ws 或 /api/ros_ws，这里保持与 rosHealth 一致
    return request.post<{ status: string; sent_to: string }>('/ros_ws/control', data);
}