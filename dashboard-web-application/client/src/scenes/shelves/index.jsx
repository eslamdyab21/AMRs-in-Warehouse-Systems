import React from "react";
import { Box, useTheme } from "@mui/material";
import { useGetShelvesQuery } from "state/api";
import Header from "components/Header";
import { DataGrid } from "@mui/x-data-grid";


const Shelves = () => {
  const theme = useTheme();
  const { data, isLoading } = useGetShelvesQuery();
  console.log("data", data);
  

  const columns = [
    {
      field: "ShelfID",
      headerName: "ID",
      flex: 0.5,
    },
    {
      field: "ProductID",
      headerName: "Product ID",
      flex: 0.5,
    },
    {
      field: "HavingOrder",
      headerName: "Having Order",
      flex: 0.5,
    },
    {
      field: "LocationX",
      headerName: "Location X",
      flex: 0.4,
    },
    {
      field: "LocationY",
      headerName: "Location Y",
      flex: 0.4,
    },
  ];

  return (
    <Box m="1.5rem 2.5rem">
      <Header title="SHELVES" subtitle="List of Shelves" />
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
          getRowId={(row : any) => row.ShelfID}
          rows={data || []}
          columns={columns}
        />
      </Box>
    </Box>
  );
};

export default Shelves;
