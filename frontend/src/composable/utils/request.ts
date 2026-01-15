// frontend/src/utils/request.ts
import axios from 'axios'

// 创建 axios 实例
const service = axios.create({
    // 后端接口的基础地址，开发环境通常是 http://localhost:8000/api
    baseURL: 'http://localhost:8000/api',
    timeout: 5000 // 请求超时时间：5秒
})

// 请求拦截器（可扩展：例如添加 Token）
service.interceptors.request.use(
    (config) => {
        return config
    },
    (error) => {
        return Promise.reject(error)
    }
)

// 响应拦截器（可扩展：统一处理错误）
service.interceptors.response.use(
    (response) => {
        return response.data
    },
    (error) => {
        console.error('请求错误:', error)
        return Promise.reject(error)
    }
)

export default service