import React , { useEffect } from "react";
import { Box, useTheme } from "@mui/material";
import { useGetMap2dQuery } from "state/api";
import Header from "components/Header";
import { DataGrid } from "@mui/x-data-grid";
import { Title } from "@mui/icons-material";



var board = [];
var rows = 9;
var columns = 9;
let r
let c
let x = 0
let y = 0
let api_url = "http://localhost:5000/api/map2d"
function Map2d() {
    const theme = useTheme();

    // let data = funcName(api_url, board)

    setInterval(function(){
        // funcName(api_url, board)
        Fill(board)
    }, 500);

    useEffect(() => {
        if (board != undefined){
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
            


    console.log('in Fill 0000')
    
    
    return (
        <div id="board" >
        </div>
      );
    
    
}


function sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}

async function Fill(board){
    // for(let k=0; k<10; k++){

        let data = funcName(api_url, board)
    // }
}

async function fill_board(data, board){
    console.log('in fill_board')

    for (let i=0; i<2; i++){
        let data_section = data[i]
        for (let j=0; j < data_section.length; j++){

            console.log(data_section[j])
            if ('robotid' in data_section[j]){
                console.log(data_section[j]['robotid'])
                r = data_section[j]['currentlocation_y']
                c = data_section[j]['currentlocation_x']
                console.log(r,c)
                
                x = parseInt(r,10)
                y = parseInt(c,10)
                

                let tile = board[x][y];
                tile.innerText = data_section[j]['robotid'];

                // console.log(x,y)

            }
        }
    }

    // await sleep(500);
}



async function funcName(url, board){
    const response = await fetch(url);
    var data = await response.json();

    fill_board(data, board)
    return data
}

export default Map2d;
