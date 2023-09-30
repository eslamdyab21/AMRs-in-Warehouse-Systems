FROM node:16.20-alpine3.17

RUN mkdir -p /home/app

COPY dashboard-web-application/client/build /home/app

EXPOSE 3000

RUN npm install -g serve

CMD ["serve", "/home/app/"]