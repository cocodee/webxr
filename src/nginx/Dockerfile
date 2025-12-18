FROM nginx:alpine

# 安装 OpenSSL
RUN apk add --no-cache openssl

# 创建 SSL 目录
RUN mkdir -p /etc/nginx/ssl

# 生成自签名证书
RUN openssl req -x509 -nodes -days 3650 -newkey rsa:2048 \
    -keyout /etc/nginx/ssl/nginx-selfsigned.key \
    -out /etc/nginx/ssl/nginx-selfsigned.crt \
    -subj "/C=US/ST=State/L=City/O=Organization/CN=localhost"

# 1. 复制 Nginx 配置文件
COPY default.conf /etc/nginx/conf.d/default.conf

# 2. 复制静态网页文件到镜像中
COPY static /usr/share/nginx/html

EXPOSE 443 80