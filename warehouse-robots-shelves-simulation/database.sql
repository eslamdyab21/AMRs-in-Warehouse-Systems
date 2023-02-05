USE AMR_Warehouse;
DELETE FROM Robots;
------------------------------------------------

USE AMR_Warehouse;
SELECT * FROM Products;
SELECT * FROM Shelves;
SELECT * FROM Robots;

USE AMR_Warehouse;
SHOW COLUMNS FROM Shelves;

USE AMR_Warehouse;
DROP TABLE Shelves;

USE AMR_Warehouse;
CREATE TABLE Shelves(
    ShelfID VARCHAR(5) NOT NULL PRIMARY KEY,
    LocationX INT NOT NULL CHECK(LocationX BETWEEN 0 AND 20),
    LocationY INT NOT NULL CHECK(LocationY BETWEEN 0 AND 20),
    ProductID VARCHAR(5) NOT NULL, -- The product that the shelf stores

    -- Creating relations
    FOREIGN KEY (ProductID) REFERENCES Products(ProductID)
);

------------------------------------------------
USE AMR_Warehouse;

ALTER TABLE Robots
MODIFY COLUMN CurrentLocationX INT NOT NULL CHECK(CurrentLocationX BETWEEN 0 AND 20);
MODIFY COLUMN CurrentLocationY INT NOT NULL CHECK(CurrentLocationY BETWEEN 0 AND 20);

USE AMR_Warehouse;
CREATE TABLE Robots(
    RobotID VARCHAR(5) NOT NULL PRIMARY KEY,
    Speed INT NOT NULL CHECK(Speed >= 0),

    -- Locations
    CurrentLocationX INT NOT NULL CHECK(CurrentLocationX BETWEEN 0 AND 20),
    CurrentLocationY INT NOT NULL CHECK(CurrentLocationY BETWEEN 0 AND 20),
    NextLocationX INT CHECK(NextLocationX BETWEEN 0 AND 20),
    NextLocationY INT CHECK(NextLocationY BETWEEN 0 AND 20),

    -- Information about the shelf
    ShelfID VARCHAR(5),
    CostToShelf INT NOT NULL CHECK(CostToShelf >= 0),

    -- Creating relations
    FOREIGN KEY (ShelfID) REFERENCES Shelves(ShelfID)
);