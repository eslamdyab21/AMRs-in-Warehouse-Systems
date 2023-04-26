const mysql = require('mysql2');
const dotenv = require("dotenv");
const { Client } = require('pg');

class database{
    /*
    database class is responsible for quering (reading/writting/updating) relevant 
    data from the database to the api.
    */

	//constructor
	constructor(){
		this.nothing = 1
	}
	
	//methods
    connect_to_postgress_db(){

        dotenv.config({ path: 'routes/api/.env' })
        this.db = new Client({
            host : process.env.POSTGRES_HOST,
            user : process.env.POSTGRES_USER,
            password : process.env.POSTGRES_PASSWORD,
            database : process.env.POSTGRES_DATABASE,
            port: process.env.POSTGRES_PORT,
          });

        this.db.connect((err) => {
            if (err){
                console.log('problem connecting to database....')
                throw err
            }
            console.log(`connected to ${process.env.MYSQL_DATABASE} database`)
        })

        return this.db
    }


    
    connect_to_mysql_db(){
        /*
        connect_to_db function is responsible of establishing the connection between the database and this python code
        using a cursor and defining the database which we need to be in use
        */

        dotenv.config()
        this.db = mysql.createConnection({
            host : process.env.MYSQL_HOST,
            user : process.env.MYSQL_USER,
            password : process.env.MYSQL_PASSWORD,
            database : process.env.MYSQL_DATABASE
        })

        this.db.connect((err) => {
            if (err){
                console.log('problem connecting to database....')
                throw err
            }
            console.log(`connected to ${process.env.MYSQL_DATABASE} database`)
        })

        return this.db
    }

    get_all_robots(){
        let sql_query_res
        const sql_query = 'SELECT * FROM Robots'
        const query = this.db.query(sql_query, (err, resluts) => {
            if (err) throw err
            sql_query_res =  resluts
            console.log(sql_query_res)
        })

        console.log(sql_query_res)
        return sql_query_res
    }

}

module.exports = database