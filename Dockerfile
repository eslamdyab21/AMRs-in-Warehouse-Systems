FROM node:13-alpine

RUN mkdir -p /home/app

COPY dashboard-web-application/client/build /home/app

EXPOSE 5000

RUN npm install -g serve

CMD ["serve", "-s /home/app/build/"]