const express = require('express');
const Database = require('./Database')

const router = express.Router();

/* DATABASE */
const database = new Database()
db = database.connect_to_db()


// Get All Robots
router.get('/', (req, res) => {
    res.setHeader('Access-Control-Allow-Origin', req.headers.origin)
    const sql_query = 'SELECT * FROM Robots'
    const query = db.query(sql_query, (err, resluts) => {
        if (err) throw err
        res.json(resluts)
    })
});

module.exports = router;