import React from "react";
import { Box, useTheme } from "@mui/material";
import { useGetOrdersQuery } from "state/api";
import Header from "components/Header";
import { DataGrid } from "@mui/x-data-grid";


function Data5(){
  const { data } = useGetOrdersQuery();
  console.log("data", data);
  return data
}


const Orders = () => {
  const theme = useTheme();
  const { data, isLoading } = useGetOrdersQuery();
  console.log("data", data);
  Data5()
  // console.log("data", data['quantity']);
  

  const columns = [
    {
      field: "orderid",
      headerName: "Order ID",
      flex: 0.5,
    },
    {
      field: "productid",
      headerName: "Product ID",
      flex: 0.5,
    },
    {
      field: "quantity",
      headerName: "Quantity",
      flex: 0.5,
    },
    {
      field: "shelfid",
      headerName: "Shelf ID",
      flex: 0.4,
    },
    {
      field: "orderproductstatus",
      headerName: "Order-Product Status",
      flex: 0.4,
    },
    {
      field: "orderdate",
      headerName: "Order Date-Time",
      flex: 0.4,
    },
  ];

  return (
    <Box m="1.5rem 2.5rem">
      <Header title="ORDERS" subtitle="Monitoring Orders" />
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
          getRowId={(row : any) => row.orderid+row.productid}
          rows={data || []}
          columns={columns}
        />
      </Box>
    </Box>
  );
};

export default Orders;
