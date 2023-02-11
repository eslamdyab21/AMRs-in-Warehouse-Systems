const mysql = require('mysql2');
const dotenv = require("dotenv");

class database{
    /*
    database class is responsible for quering (reading/writting/updating) relevant 
    data from the database to the api.
    */

	//constructor
	constructor(){
		this.connect_to_db()
	}
	
	//methods
    connect_to_db(){
        /*
        connect_to_db function is responsible of establishing the connection between the database and this python code
        using a cursor and defining the database which we need to be in use
        */

        dotenv.config()
        const db = mysql.createConnection({
            host : process.env.MYSQL_HOST,
            user : process.env.MYSQL_USER,
            password : process.env.MYSQL_PASSWORD,
            database : process.env.MYSQL_DATABASE
        })

        db.connect((err) => {
            if (err){
                console.log('problem connecting to database....')
                throw err
            }
            console.log(`connected to ${process.env.MYSQL_DATABASE} database`)
        })
    }

}

module.exports = database