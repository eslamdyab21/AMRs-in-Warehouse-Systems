const express = require('express');
const Database = require('./Database')

const router = express.Router();

/* DATABASE */
const database = new Database()
db = database.connect_to_db()


// Get All Shelves
router.get('/', (req, res) => {
    const sql_query = 'SELECT * FROM Shelves'
    const query = db.query(sql_query, (err, resluts) => {
        if (err) throw err
        console.log(resluts)
        res.json(resluts)
    })
});

module.exports = router;