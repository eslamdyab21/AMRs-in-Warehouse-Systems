This is a documentation for issue #48, the Docker with databases.
We invistgate the advantages that Docker may offer.


Docker is a software platform that allows you to build, test, and deploy applications quickly. Docker packages software into standardized units called containers that have everything the software needs to run including libraries, system tools, code, and runtime. 

Using Docker, you can quickly deploy and scale applications into any environment and know your code will run.


First you need to install Docker on your host machine. then build the Docker file in the `docker-with-databases/mysql-server` folder
```bash
❯ ls
Dockerfile  testing_AMRs.sql

❯ sudo docker build -t mysql_server_db 
  --build-ar MYSQL_ROOT_PASSWORD=$MYSQL_ROOT_PASSWORD .
```
And it will start building.....

<br>

Lets take a look into the Docker file then discuss the previous build command.
```sql
FROM mysql:latest

ARG MYSQL_ROOT_PASSWORD

ENV MYSQL_ROOT_PASSWORD = $MYSQL_ROOT_PASSWORD

COPY testing_AMRs.sql /docker-entrypoint-initdb.d/
```
First we specify the image we will use from docker hub and the version, here we specified mysql in the latest version.

Then `ARG` command is used to used the env variable `MYSQL_ROOT_PASSWORD` from the `.env` file, then at the 3rd line the env variable is exported to the docker container env.

Lastly we copy the `testing_AMRs.sql` file which contains the database specifications and tables, we copy it into `/docker-entrypoint-initdb.d/` to have the database created automatically when running the container.


<br>

Note that you need to create an `.env` and add the env variable value `MYSQL_ROOT_PASSWORD` in it file and source it.
example of `.env` file
```python
MYSQL_ROOT_PASSWORD=password
```
source it to the env variables exported
```bash
❯ source .env
```


<br>

After the build, the docker image is created
```bash
❯ sudo docker images
REPOSITORY        TAG       IMAGE ID       CREATED          SIZE
mysql_server_db   latest    b59b55c4a7c2   28 minutes ago   531MB
```

Now we can run it and access the db as normal, 
```bash
❯ sudo docker run mysql_server_db 
2023-04-17 03:51:52+00:00 [Note] [Entrypoint]: Entrypoint script for MySQL Server 8.0.32-1.el8 started.
2023-04-17 03:51:52+00:00 [Note] [Entrypoint]: Switching to dedicated user 'mysql'
2023-04-17 03:51:52+00:00 [Note] [Entrypoint]: Entrypoint script for MySQL Server 8.0.32-1.el8 started.
2023-04-17 03:51:53+00:00 [Note] [Entrypoint]: Initializing database files
2023-04-17T03:51:53.050772Z 0 [Warning] [MY-011068] [Server] The syntax '--skip-host-cache' is deprecated and will be removed in a future release. Please use SET GLOBAL host_cache_size=0 instead.
2023-04-17T03:51:53.050872Z 0 [System] [MY-013169] [Server] /usr/sbin/mysqld (mysqld 8.0.32) initializing of server in progress as process 80
2023-04-17T03:51:53.115809Z 1 [System] [MY-013576] [InnoDB] InnoDB initialization has started.
```

And now the container is up and running
```bash
❯ sudo docker container ls
CONTAINER ID   IMAGE             COMMAND                  CREATED         STATUS         PORTS                 NAMES
9351d88df61c   mysql_server_db   "docker-entrypoint.s…"   2 minutes ago   Up 2 minutes   3306/tcp, 33060/tcp   cool_chatterjee
```

To access the terminal of the container to start interacting with mysql
```bash
❯ docker exec -it 9351d88df61c /bin/bash
```