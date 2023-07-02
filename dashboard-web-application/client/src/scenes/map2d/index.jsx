import React , { useEffect } from "react";
// import { useTheme } from "@mui/material";




var board = [];
var rows = 15;
var columns = 15;
let r
let c
let x = 0
let y = 0
let api_url = "http://localhost:5000/api/map2d"
let tile

function Map2d() {
    // const theme = useTheme();


    setInterval(function(){
        Fill(board)
    }, 500);


    useEffect(() => {
        if (board !== undefined){
            board = []
        }

        for (let r = 0; r < rows; r++) {
            let row = [];

            for (let c = 0; c < columns; c++) {
                let tile = document.createElement("div");

                tile.id = r.toString() + "-" + c.toString();
                document.getElementById("board").append(tile);
                row.push(tile);
            }

            board.push(row);
        }
        
        Fill(board)
        
        }, []);

    
    return (
        <div id="board" >
        </div>
      );
    
    
}


// function sleep(ms) {
//     return new Promise(resolve => setTimeout(resolve, ms));
// }


async function Fill(board){
    get_data_backend(api_url, board)
}


function clear_board(board){
    for (let r = 0; r < rows; r++) {
        for (let c = 0; c < columns; c++) {
            let tile = board[r][c];
            tile.innerText = " ";
        }
    }

    return board

}



async function fill_board(data, board){

    board = clear_board(board)
    for (let i=0; i<2; i++){
        let data_section = data[i]
        for (let j=0; j < data_section.length; j++){

            // console.log(data_section[j])
            if ('robotid' in data_section[j]){
                // console.log(data_section[j]['robotid'])
                c = data_section[j]['currentlocation_y']
                r = data_section[j]['currentlocation_x']
                
                x = parseInt(r,10)
                y = parseInt(c,10)
                

                tile = board[x][y];
                if (tile.innerText === undefined){
                    tile.innerText = data_section[j]['robotid'];
                }
                else {
                    tile.innerText = data_section[j]['robotid'] + tile.innerText;
                }
                
            }

            else if ('shelfid' in data_section[j]){
                c = data_section[j]['location_y']
                r = data_section[j]['location_x']
                
                x = parseInt(r,10)
                y = parseInt(c,10)

                tile = board[x][y];
                if (tile.innerText === undefined){
                    tile.innerText = data_section[j]['shelfid'];
                }
                else {
                    tile.innerText = tile.innerText + data_section[j]['shelfid'];
                }
            }
        }
    }

    // await sleep(500);
}



async function get_data_backend(url, board){
    const response = await fetch(url);
    var data = await response.json();

    fill_board(data, board)
    return data
}

export default Map2d;
