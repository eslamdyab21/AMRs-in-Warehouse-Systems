# language: dockerfile

FROM postgres:15.2

ARG POSTGRES_USER
ENV POSTGRES_USER $POSTGRES_USER

ARG POSTGRES_PASSWORD
ENV POSTGRES_PASSWORD $POSTGRES_PASSWORD

ARG POSTGRES_DB
ENV POSTGRES_DB $POSTGRES_DB

EXPOSE 5432

RUN mkdir -p /home/system-data

RUN ls

RUN ls /home

COPY e-commerce-website-data.sql /home/system-data/

COPY docker-with-databases/postgres-database.sql /docker-entrypoint-initdb.d/