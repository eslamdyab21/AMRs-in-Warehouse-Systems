# language: dockerfile

FROM postgres:15.2

ARG POSTGRES_PASSWORD
ENV POSTGRES_PASSWORD $POSTGRES_PASSWORD


EXPOSE 5432

COPY postgres-database-tables.sql /docker-entrypoint-initdb.d/