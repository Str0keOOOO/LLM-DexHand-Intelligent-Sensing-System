import axios from 'axios'

// 创建 axios 实例
const request = axios.create({
    baseURL: 'http://localhost:8000/api',
    timeout: 5000 // 请求超时时间：5秒
})

// 请求拦截器（可扩展：例如添加 Token）
request.interceptors.request.use(
    (config) => {
        return config
    },
    (error) => {
        return Promise.reject(error)
    }
)

// 响应拦截器（可扩展：统一处理错误）
request.interceptors.response.use(
    (response) => {
        return response.data
    },
    (error) => {
        console.error('请求错误:', error)
        return Promise.reject(error)
    }
)

export default request