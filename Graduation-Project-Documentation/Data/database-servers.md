> This is a documentation for issue #36, the comparison between database servers.

In this issue we try to choose the best server according to performance. We choose 3 popular servers to compare between, MySQL, MSSQL (Microsoft SQL Server), and PostgreSQL.

In each server we executed the same database (our database) with the same tables, columns, constraints, and triggers. Also, we run all the servers on the same PC to ensure same hardware specifications. [You can check the codes here.](https://github.com/eslamdyab21/AMRs-in-Warehouse-Systems/tree/feature/data/36/database-servers)

For each server we'll execute 2 main statements, INSERT, and UPDATE, as they give us much time while integrating with our Python code compared to Python's execution speed. Also, we'll execute the same statements in the three servers to avoid many variables while evaluating the performance. So, let's explore the results!

1. MySQL Server
   - INSERT statement
     - Query
        ```
        mysql> INSERT INTO Products VALUES('P1', 1500, 100); 
        ```
      - Results
        ```
        Query OK, 1 row affected (0.14 sec)
        ```
    - UPDATE statement
      - Query
        ```
        mysql> UPDATE Products SET ItemsInStock = 95 WHERE ProductID = 'P1';
        ```
      - Results
        ```
        Query OK, 1 row affected (0.18 sec)
        Rows matched: 1  Changed: 1  Warnings: 0
        ```

2. MSSQL Server
    - INSERT statement
      - Query
        ```
        1> INSERT INTO Products VALUES('P1', 1500, 100);
        2> GO
        ```
      - Results
        ```
        SQL Server parse and compile time: 
        CPU time = 0 ms, elapsed time = 3 ms.
        SQL Server parse and compile time:
        CPU time = 0 ms, elapsed time = 0 ms.

        SQL Server Execution Times:
        CPU time = 0 ms,  elapsed time = 30 ms.

        (1 rows affected)
        ```
    - UPDATE statement
      - Query
        ```
        1> UPDATE Products SET ItemsInStock = 95 WHERE ProductID = 'P1';
        2> GO
        ```
      - Results
        ```
        SQL Server parse and compile time: 
        CPU time = 0 ms, elapsed time = 36 ms.
        SQL Server parse and compile time:
        CPU time = 0 ms, elapsed time = 0 ms.

        SQL Server Execution Times:
        CPU time = 0 ms,  elapsed time = 0 ms.

        (1 rows affected)
        ```

3. PostgreSQL Server
    - INSERT statement
      - Query
        ```
        amr_warehouse=# EXPLAIN ANALYZE INSERT INTO Products VALUES('P1', 1500, 100);
        ```
      - Results
        ```
                                                QUERY PLAN
        ------------------------------------------------------------------------------------------------
        Insert on products  (cost=0.00..0.01 rows=0 width=0) (actual time=0.059..0.059 rows=0 loops=1)
        ->  Result  (cost=0.00..0.01 rows=1 width=32) (actual time=0.001..0.001 rows=1 loops=1)
        Planning Time: 0.037 ms
        Execution Time: 0.297 ms
        (4 rows)
        ```
    - UPDATE statement
        - Query
            ```
            amr_warehouse=# EXPLAIN ANALYZE UPDATE Products SET ItemsInStock = 95 WHERE ProductID = 'P1';
            ```
        - Results
            ```
                                                                    QUERY PLAN
            -------------------------------------------------------------------------------------------------------------------------------
            Update on products  (cost=0.15..8.17 rows=0 width=0) (actual time=0.138..0.138 rows=0 loops=1)
            ->  Index Scan using products_pkey on products  (cost=0.15..8.17 rows=1 width=10) (actual time=0.053..0.055 rows=1 loops=1)
                    Index Cond: ((productid)::text = 'P1'::text)
            Planning Time: 0.200 ms
            Execution Time: 0.188 ms
            (5 rows)
            ```
<hr>

To summarize, here's a table that shows the time (in msec) of each statement for the three servers:

<p align="center">

| Server / Statement        | INSERT           | UPDATE  |
| ------------------------- |:-------------:| -----:|
| MySQL                     | 140           | 180   |
| MSSQL                     | 33            | 36    |
| PostgreSQL                | 0.33          | 0.38  |

</p>

This table tells us that PostgreSQL takes the least planning and execution time among the three servers, and for the same queries MySQL takes around __400__ times this time!