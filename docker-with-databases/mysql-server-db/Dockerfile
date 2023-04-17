FROM mysql:latest

ARG MYSQL_ROOT_PASSWORD
ENV MYSQL_ROOT_PASSWORD = $MYSQL_ROOT_PASSWORD

COPY testing_AMRs.sql /docker-entrypoint-initdb.d/