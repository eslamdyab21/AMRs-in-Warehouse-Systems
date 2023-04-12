> This is a documentation for issue #36, the comparison between database servers.

<hr>

In this issue we try to choose the best server according to performance. We choose 3 popular servers to compare between, MySQL, MSSQL (Microsoft SQL Server), and PostgreSQL.

In each server we executed the same database (our database) with the same tables, columns, constraints, and triggers. Also, we run all the servers on the same PC to ensure same hardware specifications. [You can check the codes here.](https://github.com/eslamdyab21/AMRs-in-Warehouse-Systems/tree/feature/data/36/database-servers-comparison/codes)

For each server we'll execute 2 main statements, INSERT, and UPDATE, as they give us much time while integrating with our Python code compared to Python's execution speed. Also, we'll execute the same statements in the three servers to avoid many variables while evaluating the performance. So, let's explore the results!

1. MySQL Server
   - INSERT statement
     - Query
        ```sql
        mysql> INSERT INTO Products VALUES('P1', 1500, 100); 
        ```
      - Results
        ```
        Query OK, 1 row affected (0.14 sec)
        ```
    - UPDATE statement
      - Query
        ```sql
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
        ```sql
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
        ```sql
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
        ```sql
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
            ```sql
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

<ul>

| Server / Statement        | INSERT           | UPDATE  |
| ------------------------- |:-------------:   | -----:  |
| MySQL                     | 140              | 180     |
| MSSQL                     | 33               | 36      |
| PostgreSQL                | 0.33             | 0.38    |

</ul>

This table tells us that PostgreSQL takes the least planning and execution time among the three servers, and for the same queries MySQL takes around __400__ times this time!

<hr>

## To get more accurate results we need to try the process more than once ##

### <u>Introduction</u>: ###

This time, we used Python to help us in the process of repeating inserting and updating records in the database.

We used three libraries to connect Python to our servers, ___mysql.connector___ for MySQL, ___pyodbc___ for MSSQL, and ___psycopg2___ for PostgreSQL. Those libraries help us in writing queries and executing them in Python.

We also used ___time___ library to calculate the time which each query takes, and in order to avoid __caching__ in the database, we used ___random___ library to generate random records to insert and update. Besides, we used ___Pandas___ to analyze the data we get.

> [If you want to take a look on the whole process, click here to get the notebook and see the results.](https://github.com/eslamdyab21/AMRs-in-Warehouse-Systems/blob/feature/data/36/database-servers-comparison/servers-performance.ipynb)

We repeat INSERT and UPDATE queries __200__ times, every time we insert a record, it is a different record, so we insert 200 different records to avoid caching and get better results. Every time we insert, the database should be empty as each query sees the same environment every time, so we delete the record we just insert after calculating the time it takes.

As for the records themselves, they're different as mentioned above, but they all have the same length in order to not have many variables in our experiment. Besides, all the records have been inserted in the 3 servers __in order__.

Finally, we get statistics of the results we gathered.

<br>

### <u>The steps of the process</u>: ###

1. Setting a dataframe to contain all the time results (in msec). It has 6 columns and an index. The index is the Statement ID, it starts with 1 to 200, and the 6 columns are INSERT and UPDATE query for the 3 servers. Here is a screenshot of the dataframe before storing any results:

    <ul>
    <p align="center">
    <img src="https://user-images.githubusercontent.com/70551007/231333243-11708f61-cae7-4815-90e3-e7163eeab1e2.png">
    </p>
    </ul>
  
2. Preparing the random data that we want to insert. We want to insert data into the __Products__ table, so we prepared information about _ProductID_, _Price_, and _ItemsInStock_. Here is a screenshot of the dataframe we generated randomly:

    <ul>
    <p align="center">
    <img src="https://user-images.githubusercontent.com/70551007/231333331-b8ef759a-425b-4ce3-b243-5282f6646f78.png">
    </p>
    </ul>

3. Setting the connections between MySQL, MSSQL, and PostgreSQL with Python, each at a time, to start the process of inserting and updating. This was done in 2 phases, a phase to insert a record, and another phase to update a random record, and calculate the time both take.

    ```python
    # For evaluating INSERT performance in MySQL
    for i in range(200):
        product_id, price, items_in_stock = products_data.iloc[i]
        add_record = """INSERT INTO Products VALUES('{}', {}, {});""".format(product_id, price, items_in_stock)

        start_time = time.time()
        cursor.execute(add_record)
        end_time = time.time()
        total_time = end_time - start_time
        total_time_in_msec = total_time*1000
        servers_performance.iloc[i, 0] = total_time_in_msec

        cursor.execute("""DELETE FROM Products;""")

        time.sleep(0.5)
    ```
    For the INSERT statement, we only need to retrieve the random _ProductID_, _Price_, and _ItemsInStock_ from the dataframe we made, insert the record, then evaluate the time it takes, and finally delete the record from the database to make it empty for the next record.

    ```python
    # For evaluating UPDATE performance in MySQL
    for i in range(200): # Fill the database with data
        product_id, price, items_in_stock = products_data.iloc[i]
        add_record = """INSERT INTO Products VALUES('{}', {}, {});""".format(product_id, price, items_in_stock)
        cursor.execute(add_record)
        connection.commit()

    for i in range(100): # Update randomly
        new_items_in_stock = 50
        product_id = products_data["ProductID"][i+1]
        update_record = """UPDATE Products SET ItemsInStock = {} WHERE ProductID = '{}';""".format(new_items_in_stock, product_id)

        start_time = time.time()
        cursor.execute(update_record)
        end_time = time.time()
        total_time = end_time - start_time
        total_time_in_msec = total_time*1000
        servers_performance.iloc[i, 1] = total_time_in_msec
        time.sleep(0.5)
    ```

    As for the UPDATE statement, we first fill the database with all the records we generate, then pick a random _ProductID_  and update its _ItemsInStock_ with 50. We choose this number to be out of the _ItemsInStock_ range, in order to be a new value to be written in the database.

4. After filling the dataframe with the results, we get some statistics about them.
   - Here is a screenshot of some results
   
      <ul>
      <p align="center">
      <img src="https://user-images.githubusercontent.com/70551007/231333453-534abee7-2361-4220-be09-db91f15d9347.png">
      </p>
      </ul>
       
   - Here is a screenshot of the statistics we got

      <ul>
      <p align="center">
      <img src="https://user-images.githubusercontent.com/70551007/231333655-f6c6b29c-46cb-4d9d-9f21-ed6bd8d54154.png">
      </p>
      </ul>

5. Insights
   - The standard deviation of PostgreSQL for INSERT and UPDATE statement is the lowest among the three servers, which means that the data points are less spread out.
   - PostgreSQL also has the lowest maximum result for INSERT and UPDATE, `2.3` and `4` msec respectively, while the maximum of the remaining servers exceeds `30` msec.
