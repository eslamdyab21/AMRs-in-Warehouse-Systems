import express from "express";
import mysql from "mysql2";
import bodyParser from "body-parser";
import cors from "cors";
import dotenv from "dotenv";
import helmet from "helmet";
import morgan from "morgan";
//import clientRoutes from "./routes/client.js";
//import generalRoutes from "./routes/general.js";
//import managementRoutes from "./routes/management.js";
//import salesRoutes from "./routes/sales.js";



/* DATABASE CONNECTION */
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


/* CONFIGURATION */
const app = express()

app.listen('5000', () => {
    console.log('server started at port 5000')
})


/* ROUTES */


/* Mysql SETUP */

