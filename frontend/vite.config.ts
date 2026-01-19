import {fileURLToPath, URL} from 'node:url'

import {defineConfig} from 'vite'
import vue from '@vitejs/plugin-vue'
import vueDevTools from 'vite-plugin-vue-devtools'

// https://vite.dev/config/
export default defineConfig({
    plugins: [
        vue(),
        vueDevTools(),
    ],
    resolve: {
        alias: {
            '@': fileURLToPath(new URL('./src', import.meta.url))
        },
    },
    // --- 新增: 代理配置 ---
    server: {
        host: '0.0.0.0', // 允许通过 IP 访问
        proxy: {
            '/api': {
                target: 'http://localhost:8000', // 后端地址
                changeOrigin: true, // 允许跨域
                ws: true,
                // 如果后端路由里本来就有 /api 前缀，就不需要 rewrite
                // rewrite: (path) => path.replace(/^\/api/, '')
            }
        }
    }
    // ---------------------
})