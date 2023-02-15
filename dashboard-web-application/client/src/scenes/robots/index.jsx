import React from "react";
import { Box, useTheme } from "@mui/material";
import { useGetRobotsQuery } from "state/api";
import Header from "components/Header";
import { DataGrid } from "@mui/x-data-grid";

const Robots = () => {
  const theme = useTheme();
  const { data, isLoading } = useGetRobotsQuery();
  console.log("data", data);

  const columns = [
    {
        field: "RobotID",
        headerName: "ID",
        flex: 0.5,
      },
      {
        field: "BatteryLife",
        headerName: "Battery Life",
        flex: 0.5,
      },
      {
        field: "Speed",
        headerName: "Speed",
        flex: 0.5,
      },
      {
        field: "CurrentLocationX",
        headerName: "Location X",
        flex: 0.4,
      },
      {
        field: "CurrentLocationY",
        headerName: "Location Y",
        flex: 0.4,
      },
      {
        field: "ShelfID",
        headerName: "Connect to Shelf",
        flex: 0.4,
      },
      {
        field: "Moving",
        headerName: "Moving",
        flex: 0.4,
      },

  ];

  return (
    <Box m="1.5rem 2.5rem">
      <Header title="Robots" subtitle="List of Robots" />
      <Box
        mt="40px"
        height="75vh"
        sx={{
          "& .MuiDataGrid-root": {
            border: "none",
          },
          "& .MuiDataGrid-cell": {
            borderBottom: "none",
          },
          "& .MuiDataGrid-columnHeaders": {
            backgroundColor: theme.palette.background.alt,
            color: theme.palette.secondary[100],
            borderBottom: "none",
          },
          "& .MuiDataGrid-virtualScroller": {
            backgroundColor: theme.palette.primary.light,
          },
          "& .MuiDataGrid-footerContainer": {
            backgroundColor: theme.palette.background.alt,
            color: theme.palette.secondary[100],
            borderTop: "none",
          },
          "& .MuiDataGrid-toolbarContainer .MuiButton-text": {
            color: `${theme.palette.secondary[200]} !important`,
          },
        }}
      >
        <DataGrid
          loading={isLoading || !data}
          getRowId={(row:any) => row.RobotID}
          rows={data || []}
          columns={columns}
        />
      </Box>
    </Box>
  );
};

export default Robots;
