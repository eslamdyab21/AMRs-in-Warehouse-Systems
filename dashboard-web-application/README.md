
There are two main folders, `server and client`.
- The client is the `ui/frontend` for the application
- The server is the `backend` where an `api` is created to make communication between the client side and the server side.

</br>

### Method 1, run the application on your OS directly
>make sure you installed node on your machine first


To run the client/frontend 
```bash
❯ cd client

❯ npm install

❯ npm start
```
The frontend will start at localhost at port 3000 but won't show any data because we didn't start the backend server and it's not finished yet.

</br>

To run the server/backend
```bash
❯ cd server

❯ npm install

❯ npm run dev
```
The backend will start at localhost at port 5000.

</br>

### Method 2, run the application using Docker (recommended)
>Easy one command, make sure you have docker installed 

</br>

#### Front-end

```bash
❯ docker run --name dashboard-dev-0.1 -p 3000:3000 -d eslamdyba/amrs-in-warehouse-systems:dashboard-dev-0.1
```
It will pull the image from docker-hub and start the container at localhost port 3000.

```bash
❯ docker ps
CONTAINER ID   IMAGE                                                   COMMAND                  CREATED         STATUS         PORTS                                       NAMES
99dcc198313e   eslamdyba/amrs-in-warehouse-systems:dashboard-dev-0.1   "docker-entrypoint.s…"   9 minutes ago   Up 8 minutes   0.0.0.0:3000->3000/tcp, :::3000->3000/tcp   dashboard-dev-0.1
```

![](/Graduation-Project-Documentation/Software/images/dashboard.png)

</br>

#### Back-end
```bash
❯ docker run -d --name dashboard-backend --network warehouse-network -p 5000:5000  eslamdyba/amrs-in-warehouse-systems:dashboard-backend-dev-0.0
```

It will pull the image from docker-hub and start the container at localhost port 5000. And the front-end will talk to the back-end through the `warehouse-network`, you need to create this network first
```bash
❯ docker network create warehouse-network
```

