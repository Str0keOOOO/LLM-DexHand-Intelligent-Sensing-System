import request from '@/composable/utils/request'
import type {RosHealth} from "@/composable/interfaces/Inter2Robot.ts";

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

export async function rosHealth(){
    return request.get<RosHealth>('/ros_ws/health')
}