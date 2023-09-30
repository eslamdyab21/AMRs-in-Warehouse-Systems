const express = require('express');
const Database = require('./Database')

const router = express.Router();

/* DATABASE */
const database = new Database()
// db = database.connect_to_db()
db = database.connect_to_postgress_db()

let robots_id_location
let shelves_id_locations

// Get All Robots
router.get('/', (req, res) => {
    res.setHeader('Access-Control-Allow-Origin', "*")
    const sql_query1 = 'SELECT RobotID, CurrentLocation_X, CurrentLocation_Y FROM Robots'
    const sql_query2 = 'SELECT ShelfID, location_x, location_y FROM Shelves'
    const query1 = db.query(sql_query1, (err, results1) => {
        if (err) throw err
        // res.json(results.rows)
        robots_id_location = results1.rows

        const query2 = db.query(sql_query2, (err, results2) => {
            if (err) throw err
            
            shelves_id_locations = results2.rows
            const robots_shelves_id_location = [robots_id_location, shelves_id_locations]
            // console.log(robots_shelves_id_location)

            res.json(robots_shelves_id_location)
        })
    })
    

    
    
});

module.exports = router;