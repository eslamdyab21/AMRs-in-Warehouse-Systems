const express = require('express');
const router = express.Router();

const users = [
    {
      id: 1,
      name: 'John Doe',
      email: 'john@gmail.com'
    },
    {
      id: 2,
      name: 'Bob Williams',
      email: 'bob@gmail.com'
    },
    {
      id: 3,
      name: 'Shannon Jackson',
      email: 'shannon@gmail.com'
    }
  ]

// Gets All users
router.get('/', (req, res) => res.json(users));

module.exports = router;