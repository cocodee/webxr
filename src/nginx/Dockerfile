FROM nginx:alpine

# 安装 OpenSSL 用于生成证书
RUN apk add --no-cache openssl

# 创建存放证书的目录
RUN mkdir -p /etc/nginx/ssl

# 在构建阶段自动生成自签名证书 (有效期 10 年)
# 这里的 /CN=localhost 只是为了不报错，手机端仍需信任证书
RUN openssl req -x509 -nodes -days 3650 -newkey rsa:2048 \
    -keyout /etc/nginx/ssl/nginx-selfsigned.key \
    -out /etc/nginx/ssl/nginx-selfsigned.crt \
    -subj "/C=US/ST=State/L=City/O=Organization/CN=localhost"

# 复制自定义的 Nginx 配置
COPY default.conf /etc/nginx/conf.d/default.conf

# 暴露 HTTPS 端口
EXPOSE 443