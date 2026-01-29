import request from '@/composable/utils/request'
import type { ControlCommand } from '@/composable/interfaces/Inter2Robot';


export async function sendControlCommand(data: ControlCommand) {
    return request.post<{ status: string; sent_to: string }>('/ros_ws/control', data);
}

