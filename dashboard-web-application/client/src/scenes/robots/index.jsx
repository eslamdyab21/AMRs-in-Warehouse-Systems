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
        field: "robotid",
        headerName: "ID",
        flex: 0.5,
      },
      {
        field: "batterypercentage",
        headerName: "Battery Life",
        flex: 0.5,
      },
      {
        field: "speed",
        headerName: "Speed",
        flex: 0.5,
      },
      {
        field: "currentlocation_x",
        headerName: "Location X",
        flex: 0.4,
      },
      {
        field: "currentlocation_y",
        headerName: "Location Y",
        flex: 0.4,
      },
      {
        field: "shelfid",
        headerName: "Connected to Shelf",
        flex: 0.4,
      },
      {
        field: "ischarging",
        headerName: "Charger status",
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
          getRowId={(row:any) => row.robotid}
          rows={data || []}
          columns={columns}
        />
      </Box>
    </Box>
  );
};

export default Robots;
