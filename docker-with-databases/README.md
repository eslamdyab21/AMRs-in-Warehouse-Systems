>This is a documentation for issue #48, the Docker with databases.
We investigate the advantages that Docker may offer.


Docker is a software platform that allows you to build, test, and deploy applications quickly. Docker packages software into standardized units called containers that have everything the software needs to run including libraries, system tools, code, and runtime. 

Using Docker, you can quickly deploy and scale applications into any environment and know your code will run.


First you need to install Docker on your host machine. then build the Docker file in the `docker-with-databases` folder
```bash
❯ ls
Database_pstgress.py  Logger.py                     __pycache__
Dockerfile            main.py                       README.md
logger.log            postgres-database-tables.sql  

❯ sudo docker build -t pg_server_db 
  --build-arg POSTGRES_PASSWORD=$POSTGRES_PASSWORD .
```
And it will start building.....

<br>

Lets take a look into the Docker file then discuss the previous build command.
```sql
FROM postgres:15.2

ARG POSTGRES_PASSWORD
ENV POSTGRES_PASSWORD $POSTGRES_PASSWORD

EXPOSE 5432

COPY postgres-database-tables.sql /docker-entrypoint-initdb.d/
```
First we specify the image we will use from docker hub and the version, here we specified postgres in the 15.2 version.

Then `ARG` command is used for the env variable `POSTGRES_PASSWORD` from the `.env` file, then at the 3rd line the env variable is exported to the docker container env.

Lastly we copy the `postgres-database-tables.sql` file which contains the database specifications and tables, we copy it into `/docker-entrypoint-initdb.d/` to have the database created automatically when running the container.


<br>

Note that you need to create an `.env` and add the env variable value `MYSQL_ROOT_PASSWORD` in it file and source it.
example of `.env` file
```python
POSTGRES_PASSWORD=password
```
source it to the env variables exported
```bash
❯ source .env
```


<br>

After the build, the docker image is created
```bash
❯ docker images
REPOSITORY        TAG       IMAGE ID       CREATED             SIZE
pg_server_db      latest    1ed614ee9730   About an hour ago   379MB
```

Now we can run it and access the db as normal, 
```bash
❯ docker run -it --name my-postgresdb-container -p 5433:5432 pg_server_db 
The files belonging to this database system will be owned by user "postgres".
This user must also own the server process.

.
.
.

Success. You can now start the database server using:

    pg_ctl -D /var/lib/postgresql/data -l logfile start

waiting for server to start....2023-04-26 21:48:45.690 UTC [47] LOG:  starting PostgreSQL 15.2 (Debian 15.2-1.pgdg110+1) on x86_64-pc-linux-gnu, compiled by gcc (Debian 10.2.1-6) 10.2.1 20210110, 64-bit
2023-04-26 21:48:45.780 UTC [47] LOG:  listening on Unix socket "/var/run/postgresql/.s.PGSQL.5432"
2023-04-26 21:48:46.008 UTC [50] LOG:  database system was shut down at 2023-04-26 21:48:40 UTC
2023-04-26 21:48:46.078 UTC [47] LOG:  database system is ready to accept connections
 done
server started

/usr/local/bin/docker-entrypoint.sh: running /docker-entrypoint-initdb.d/postgres-database-tables.sql
CREATE DATABASE
CREATE TABLE
CREATE TABLE
.
.
.
```

And now the container is up and running
```bash
❯ docker container ls
CONTAINER ID   IMAGE             COMMAND                  CREATED         STATUS         PORTS                 NAMES
68c5fd5ba538   pg_server_db   "docker-entrypoint.s…"   55 minutes ago   Up 54 minutes   0.0.0.0:5433->5432/tcp, :::5433->5432/tcp   my-postgresdb-container
```

<br>

To access the terminal of the container to start interacting with psql
```bash
❯ docker exec -it 68c5fd5ba538 /bin/bash
```
or we can access it from our host terminal like we normally would if postgres was instated on out host machine.
```bash
psql -h 172.17.0.2 -p 5433 -U postgres -d postgres
```
where `172.17.0.2` is the ip address of the container running postgres, and `5433` is the port which the container forwards to in our host machine.



![](/Graduation-Project-Documentation/Data/images/docker_postgres_image.png)


<br>

## Speed comparison between postgres running in docker and running natively in our host machine

We made a small script `main.py` to update entries in the database in both environments
```python
for i in range(10):
	db.update_db(S1, parameters)
	db.update_db(S2, parameters)
	db.update_db(R1, parameters)
	db.update_db(R2, parameters)
	
	time.sleep(1)
```

We can switch between the two environments by changing the .env file information
ex:
```python  
# for docker container
# POSTGRES_USER="postgres"
# POSTGRES_PASSWORD="password"
# POSTGRES_HOST="172.17.0.2"
# POSTGRES_DATABASE="postgres"
# POSTGRES_PORT=5432

  
# for host machine
POSTGRES_USER = "user"
POSTGRES_PASSWORD = "password"
POSTGRES_HOST = "localhost"
POSTGRES_DATABASE = "db_name"
POSTGRES_PORT = 5432
```

<br>

And the time info is logged in `logger.log`
```python
Docker container postgres
----------------------------------

2023-04-27 00:29:42,872: Logger: Database : connect_to_db : 0.011043310165405273 --> Connection is done
2023-04-27 00:29:42,872: Logger: Database --> postgres Database is in use
2023-04-27 00:29:42,918: Logger: Database : update_db : 0.04510235786437988 --> S2 is updated
2023-04-27 00:29:42,951: Logger: Database : update_db : 0.032486915588378906 --> S1 is updated
2023-04-27 00:29:42,962: Logger: Database : update_db : 0.010576963424682617 --> R2 is updated
2023-04-27 00:29:42,973: Logger: Database : update_db : 0.010669946670532227 --> R1 is updated
2023-04-27 00:29:42,975: Logger: Database : query_recived_order_shelfs_id : 0.0015430450439453125 --> Shelves that have recived an order: ['S1']
2023-04-27 00:29:42,975: Logger: ------------------------breaker------------------------
2023-04-27 00:29:42,975: Logger:
2023-04-27 00:29:43,995: Logger: Database : update_db : 0.0187375545501709 --> S2 is updated
2023-04-27 00:29:44,017: Logger: Database : update_db : 0.021614789962768555 --> S1 is updated
2023-04-27 00:29:44,039: Logger: Database : update_db : 0.02154254913330078 --> R2 is updated
2023-04-27 00:29:44,061: Logger: Database : update_db : 0.02161264419555664 --> R1 is updated
2023-04-27 00:29:44,063: Logger: Database : query_recived_order_shelfs_id : 0.0010480880737304688 --> Shelves that have recived an order: ['S2', 'S1']
2023-04-27 00:29:44,063: Logger: ------------------------breaker------------------------

2023-04-27 00:29:44,063: Logger:
2023-04-27 00:29:45,084: Logger: Database : update_db : 0.019466161727905273 --> S2 is updated
2023-04-27 00:29:45,106: Logger: Database : update_db : 0.02153325080871582 --> S1 is updated
2023-04-27 00:29:45,128: Logger: Database : update_db : 0.021587610244750977 --> R2 is updated
2023-04-27 00:29:45,150: Logger: Database : update_db : 0.02162766456604004 --> R1 is updated
2023-04-27 00:29:45,151: Logger: Database : query_recived_order_shelfs_id : 0.0011138916015625 --> Shelves that have recived an order: ['S2', 'S1']
2023-04-27 00:29:45,151: Logger: ------------------------breaker------------------------

2023-04-27 00:29:45,152: Logger:
2023-04-27 00:29:46,172: Logger: Database : update_db : 0.019400358200073242 --> S2 is updated
2023-04-27 00:29:46,194: Logger: Database : update_db : 0.021625518798828125 --> S1 is updated
2023-04-27 00:29:46,216: Logger: Database : update_db : 0.021471023559570312 --> R2 is updated
2023-04-27 00:29:46,238: Logger: Database : update_db : 0.021599292755126953 --> R1 is updated
2023-04-27 00:29:46,239: Logger: Database : query_recived_order_shelfs_id : 0.0008852481842041016 --> Shelves that have recived an order: ['S2', 'S1']
2023-04-27 00:29:46,240: Logger: ------------------------breaker------------------------

2023-04-27 00:29:46,240: Logger:
2023-04-27 00:29:47,261: Logger: Database : update_db : 0.01963329315185547 --> S2 is updated
2023-04-27 00:29:47,272: Logger: Database : update_db : 0.010528326034545898 --> S1 is updated
2023-04-27 00:29:47,283: Logger: Database : update_db : 0.010619878768920898 --> R2 is updated
2023-04-27 00:29:47,294: Logger: Database : update_db : 0.01054692268371582 --> R1 is updated
2023-04-27 00:29:47,295: Logger: Database : query_recived_order_shelfs_id : 0.0009572505950927734 --> Shelves that have recived an order: ['S2', 'S1']
2023-04-27 00:29:47,295: Logger: ------------------------breaker------------------------

2023-04-27 00:29:47,295: Logger:
2023-04-27 00:29:48,316: Logger: Database : update_db : 0.02054286003112793 --> S2 is updated
2023-04-27 00:29:48,338: Logger: Database : update_db : 0.0215914249420166 --> S1 is updated
2023-04-27 00:29:48,360: Logger: Database : update_db : 0.02159905433654785 --> R2 is updated
2023-04-27 00:29:48,383: Logger: Database : update_db : 0.021752595901489258 --> R1 is updated
2023-04-27 00:29:48,384: Logger: Database : query_recived_order_shelfs_id : 0.0009138584136962891 --> Shelves that have recived an order: ['S2', 'S1']
2023-04-27 00:29:48,384: Logger: ------------------------breaker------------------------

2023-04-27 00:29:48,384: Logger:
2023-04-27 00:29:49,405: Logger: Database : update_db : 0.019222736358642578 --> S2 is updated
2023-04-27 00:29:49,427: Logger: Database : update_db : 0.021681547164916992 --> S1 is updated
2023-04-27 00:29:49,449: Logger: Database : update_db : 0.02158188819885254 --> R2 is updated
2023-04-27 00:29:49,471: Logger: Database : update_db : 0.021590709686279297 --> R1 is updated
2023-04-27 00:29:49,472: Logger: Database : query_recived_order_shelfs_id : 0.0009500980377197266 --> Shelves that have recived an order: ['S2', 'S1']
2023-04-27 00:29:49,472: Logger: ------------------------breaker------------------------

2023-04-27 00:29:49,473: Logger:
2023-04-27 00:29:50,594: Logger: Database : update_db : 0.11998796463012695 --> S2 is updated
2023-04-27 00:29:50,614: Logger: Database : update_db : 0.020156383514404297 --> S1 is updated
2023-04-27 00:29:50,636: Logger: Database : update_db : 0.02180194854736328 --> R2 is updated
2023-04-27 00:29:50,658: Logger: Database : update_db : 0.021825075149536133 --> R1 is updated
2023-04-27 00:29:50,659: Logger: Database : query_recived_order_shelfs_id : 0.0006868839263916016 --> Shelves that have recived an order: ['S2', 'S1']
2023-04-27 00:29:50,659: Logger: ------------------------breaker------------------------

2023-04-27 00:29:50,659: Logger:
2023-04-27 00:29:51,681: Logger: Database : update_db : 0.020489215850830078 --> S2 is updated
2023-04-27 00:29:51,703: Logger: Database : update_db : 0.021439790725708008 --> S1 is updated
2023-04-27 00:29:51,725: Logger: Database : update_db : 0.02176046371459961 --> R2 is updated
2023-04-27 00:29:51,747: Logger: Database : update_db : 0.021596908569335938 --> R1 is updated
2023-04-27 00:29:51,749: Logger: Database : query_recived_order_shelfs_id : 0.001047372817993164 --> Shelves that have recived an order: ['S2', 'S1']
2023-04-27 00:29:51,749: Logger: ------------------------breaker------------------------

2023-04-27 00:29:51,749: Logger:
2023-04-27 00:29:54,182: Logger: Database : update_db : 1.4310884475708008 --> S2 is updated
2023-04-27 00:29:54,247: Logger: Database : update_db : 0.06476902961730957 --> S1 is updated
2023-04-27 00:29:54,265: Logger: Database : update_db : 0.017824888229370117 --> R2 is updated
2023-04-27 00:29:54,276: Logger: Database : update_db : 0.010509490966796875 --> R1 is updated
2023-04-27 00:29:54,278: Logger: Database : query_recived_order_shelfs_id : 0.0012481212615966797 --> Shelves that have recived an order: ['S2', 'S1']
2023-04-27 00:29:54,278: Logger: ------------------------breaker------------------------
2023-04-27 00:29:54,278: Logger:

  
  
  
Host machine postgres
-------------------------------------------------------------------------------------

-------------------------------------------------------------------------------------

2023-04-27 00:37:56,794: Logger: Database : connect_to_db : 0.012960195541381836 --> Connection is done
2023-04-27 00:37:56,794: Logger: Database --> AMR_Warehouse Database is in use
2023-04-27 00:37:56,856: Logger: Database : update_db : 0.0617823600769043 --> S2 is updated
2023-04-27 00:37:56,976: Logger: Database : update_db : 0.12020516395568848 --> S1 is updated
2023-04-27 00:37:57,113: Logger: Database : update_db : 0.13582110404968262 --> R2 is updated
2023-04-27 00:37:57,241: Logger: Database : update_db : 0.12758898735046387 --> R1 is updated
2023-04-27 00:37:57,241: Logger: Database : query_recived_order_shelfs_id : 0.0004088878631591797 --> Shelves that have recived an order: ['S1']
2023-04-27 00:37:57,241: Logger: ------------------------breaker------------------------

2023-04-27 00:37:57,241: Logger:
2023-04-27 00:37:58,649: Logger: Database : update_db : 0.4064934253692627 --> S2 is updated
2023-04-27 00:37:58,817: Logger: Database : update_db : 0.16718268394470215 --> S1 is updated
2023-04-27 00:37:58,943: Logger: Database : update_db : 0.12608051300048828 --> R2 is updated
2023-04-27 00:37:58,960: Logger: Database : update_db : 0.016292095184326172 --> R1 is updated
2023-04-27 00:37:58,961: Logger: Database : query_recived_order_shelfs_id : 0.001180887222290039 --> Shelves that have recived an order: ['S2', 'S1']
2023-04-27 00:37:58,962: Logger: ------------------------breaker------------------------

2023-04-27 00:37:58,962: Logger:
2023-04-27 00:37:59,982: Logger: Database : update_db : 0.01975560188293457 --> S2 is updated
2023-04-27 00:38:00,004: Logger: Database : update_db : 0.021608591079711914 --> S1 is updated
2023-04-27 00:38:00,026: Logger: Database : update_db : 0.021584510803222656 --> R2 is updated
2023-04-27 00:38:00,048: Logger: Database : update_db : 0.021549224853515625 --> R1 is updated
2023-04-27 00:38:00,050: Logger: Database : query_recived_order_shelfs_id : 0.0010538101196289062 --> Shelves that have recived an order: ['S2', 'S1']
2023-04-27 00:38:00,050: Logger: ------------------------breaker------------------------

2023-04-27 00:38:00,050: Logger:
2023-04-27 00:38:01,071: Logger: Database : update_db : 0.020287036895751953 --> S2 is updated
2023-04-27 00:38:01,093: Logger: Database : update_db : 0.021591901779174805 --> S1 is updated
2023-04-27 00:38:01,115: Logger: Database : update_db : 0.02157735824584961 --> R2 is updated
2023-04-27 00:38:01,137: Logger: Database : update_db : 0.02161097526550293 --> R1 is updated
2023-04-27 00:38:01,138: Logger: Database : query_recived_order_shelfs_id : 0.0010058879852294922 --> Shelves that have recived an order: ['S2', 'S1']
2023-04-27 00:38:01,138: Logger: ------------------------breaker------------------------

2023-04-27 00:38:01,139: Logger:
2023-04-27 00:38:02,284: Logger: Database : update_db : 0.14359426498413086 --> S2 is updated
2023-04-27 00:38:02,449: Logger: Database : update_db : 0.16519856452941895 --> S1 is updated
2023-04-27 00:38:02,586: Logger: Database : update_db : 0.13677501678466797 --> R2 is updated
2023-04-27 00:38:02,714: Logger: Database : update_db : 0.12728047370910645 --> R1 is updated
2023-04-27 00:38:02,715: Logger: Database : query_recived_order_shelfs_id : 0.0013031959533691406 --> Shelves that have recived an order: ['S2', 'S1']
2023-04-27 00:38:02,716: Logger: ------------------------breaker------------------------

2023-04-27 00:38:02,716: Logger:
2023-04-27 00:38:03,752: Logger: Database : update_db : 0.034908294677734375 --> S2 is updated
2023-04-27 00:38:03,793: Logger: Database : update_db : 0.04089784622192383 --> S1 is updated
2023-04-27 00:38:03,808: Logger: Database : update_db : 0.01476740837097168 --> R2 is updated
2023-04-27 00:38:03,841: Logger: Database : update_db : 0.032985687255859375 --> R1 is updated
2023-04-27 00:38:03,842: Logger: Database : query_recived_order_shelfs_id : 0.0004112720489501953 --> Shelves that have recived an order: ['S2', 'S1']
2023-04-27 00:38:03,842: Logger: ------------------------breaker------------------------

2023-04-27 00:38:03,842: Logger:
2023-04-27 00:38:04,864: Logger: Database : update_db : 0.021350383758544922 --> S2 is updated
2023-04-27 00:38:04,897: Logger: Database : update_db : 0.032855987548828125 --> S1 is updated
2023-04-27 00:38:04,930: Logger: Database : update_db : 0.03275632858276367 --> R2 is updated
2023-04-27 00:38:04,963: Logger: Database : update_db : 0.032774925231933594 --> R1 is updated
2023-04-27 00:38:04,963: Logger: Database : query_recived_order_shelfs_id : 0.00034332275390625 --> Shelves that have recived an order: ['S2', 'S1']
2023-04-27 00:38:04,963: Logger: ------------------------breaker------------------------

2023-04-27 00:38:04,963: Logger:
2023-04-27 00:38:05,985: Logger: Database : update_db : 0.020894527435302734 --> S2 is updated
2023-04-27 00:38:06,019: Logger: Database : update_db : 0.03288149833679199 --> S1 is updated
2023-04-27 00:38:06,052: Logger: Database : update_db : 0.03253293037414551 --> R2 is updated
2023-04-27 00:38:06,084: Logger: Database : update_db : 0.03222846984863281 --> R1 is updated
2023-04-27 00:38:06,085: Logger: Database : query_recived_order_shelfs_id : 0.0005064010620117188 --> Shelves that have recived an order: ['S2', 'S1']
2023-04-27 00:38:06,085: Logger: ------------------------breaker------------------------

2023-04-27 00:38:06,085: Logger:
2023-04-27 00:38:07,107: Logger: Database : update_db : 0.021195173263549805 --> S2 is updated
2023-04-27 00:38:07,140: Logger: Database : update_db : 0.0328831672668457 --> S1 is updated
2023-04-27 00:38:07,173: Logger: Database : update_db : 0.03303670883178711 --> R2 is updated
2023-04-27 00:38:07,238: Logger: Database : update_db : 0.06508660316467285 --> R1 is updated
2023-04-27 00:38:07,239: Logger: Database : query_recived_order_shelfs_id : 0.00040411949157714844 --> Shelves that have recived an order: ['S2', 'S1']
2023-04-27 00:38:07,239: Logger: ------------------------breaker------------------------

2023-04-27 00:38:07,239: Logger:
2023-04-27 00:38:08,278: Logger: Database : update_db : 0.03820157051086426 --> S2 is updated
2023-04-27 00:38:08,324: Logger: Database : update_db : 0.04514336585998535 --> S1 is updated
2023-04-27 00:38:08,339: Logger: Database : update_db : 0.014317035675048828 --> R2 is updated
2023-04-27 00:38:08,350: Logger: Database : update_db : 0.010596752166748047 --> R1 is updated
2023-04-27 00:38:08,351: Logger: Database : query_recived_order_shelfs_id : 0.0011816024780273438 --> Shelves that have recived an order: ['S2', 'S1']
2023-04-27 00:38:08,352: Logger: ------------------------breaker------------------------

```

There is no mush difference in speed, but docker is easier to setup and reproducible.