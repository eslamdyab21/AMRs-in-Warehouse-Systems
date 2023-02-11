const express = require('express');
const Database = require('./routes/api/Database')



/* DATABASE CONNECTION */
const database = new Database()


/* CONFIGURATION */
const app = express()

app.listen('5000', () => {
    console.log('server started at port 5000')
})


/* API ROUTES */
// app.use('/api/shelves', require('./routes/api/shelves'));
app.use('/api/robots', require('./routes/api/robots'));
// app.use('/api/products', require('./routes/api/products'));



