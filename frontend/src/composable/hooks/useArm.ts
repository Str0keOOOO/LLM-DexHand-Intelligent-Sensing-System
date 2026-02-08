import {ref, onMounted, onUnmounted} from 'vue';
import {armApi} from '@/composable/api/Chat2Arm';
import {ElMessage} from 'element-plus';

// TODO 改成websocket
export function useArm() {
    const isConnected = ref(false);
    const jointPositions = ref<number[]>([]);
    const currentLine = ref(0);
    const isRunning = ref(false);
    let timer: number | null = null;

    // 基础操作
    const connect = async () => {
        try {
            await armApi.connect();
            isConnected.value = true;
            ElMessage.success('机械臂已连接');
        } catch (e) {
            ElMessage.error('连接失败');
        }
    };

    const init = async () => {
        try {
            await armApi.initRobot();
            ElMessage.success('机械臂初始化序列已启动');
        } catch (e) {
            ElMessage.error('初始化失败');
        }
    };

    // 夹爪控制
    const controlGripper = async (open: boolean) => {
        try {
            open ? await armApi.openGripper() : await armApi.closeGripper();
            ElMessage.success(open ? '夹爪已打开' : '夹爪已关闭');
        } catch (e) {
            ElMessage.error('夹爪控制失败');
        }
    };

    // 脚本调用
    const runScript = async (name: string) => {
        try {
            await armApi.callTrackScript({script_name: name});
            ElMessage.success(`开始执行脚本: ${name}`);
        } catch (e) {
            ElMessage.error('脚本调用失败');
        }
    };

    // 轮询状态
    const refreshStatus = async () => {
        try {
            const [pos, line, state] = await Promise.all([
                armApi.getActualJointPos(),
                armApi.getCurrentLine(),
                armApi.checkProgramState()
            ]);
            jointPositions.value = pos.joints || [];
            currentLine.value = line.line;
            isRunning.value = (state as any).is_running;
        } catch (e) {
            console.error('获取状态失败');
        }
    };

    onMounted(() => {
        timer = window.setInterval(refreshStatus, 1000); // 每秒刷新一次状态
    });

    onUnmounted(() => {
        if (timer) clearInterval(timer);
    });

    return {
        isConnected, jointPositions, currentLine, isRunning,
        connect, init, controlGripper, runScript
    };
}