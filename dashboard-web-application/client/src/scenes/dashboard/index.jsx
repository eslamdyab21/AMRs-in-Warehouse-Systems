import React from "react";
import FlexBetween from "components/FlexBetween";
import Header from "components/Header";
// require('dotenv').config()

import {
  DownloadOutlined,
  Email,
  PointOfSale,
  PersonAdd,
  Traffic,
} from "@mui/icons-material";
import {
  Box,
  Button,
  Typography,
  useTheme,
  useMediaQuery,
} from "@mui/material";
// import { DataGrid } from "@mui/x-data-grid";
import BreakdownChart from "components/BreakdownChart";
import OverviewChart from "components/OverviewChart";
import { useGetOrdersQuery } from "state/api";
import { useGetDashboardQuery } from "state/api";
import StatBox from "components/StatBox";

// dotenv.config({ path: '../.env.local' })
let baseQuery = process.env.REACT_APP_BASE_URL


let api_url = baseQuery+'/api/orders'
console.log(api_url)
// let api_url = "http://localhost:5000/api/orders"

async function get_data_backend(url){
  const response = await fetch(url);
  var data = await response.json();

  return data
}



function Dashboard(){

  const theme = useTheme();
  const isNonMediumScreens = useMediaQuery("(min-width: 1200px)");
  const { data_orders, isLoading2 } = useGetOrdersQuery();
  const { data, isLoading } = useGetDashboardQuery();
  console.log(isLoading)

  console.log("data_orders", data_orders);

  const printAddress = async () => {
    const data_orders = await get_data_backend(api_url)
    console.log("data_orders", data_orders);
    console.log("isLoading2", isLoading2);

    let data_grid = document.getElementById("dashboard_table")
    console.log("data_grid", data_grid)
  };

  setInterval(function(){
    printAddress()

  }, 5000);

  // setInterval(function(){
  //   // const { data_orders, isLoading2 } = useGetOrdersQuery();
    

  //   <DataGrid
  //           loading={isLoading2 || !data_orders}
  //           getRowId={(row : any) => row.orderid+row.productid}
  //           rows={data_orders || []}
  //           columns={columns}
  //   />

  // }, 500);

  // const columns = [
  //   {
  //     field: "orderid",
  //     headerName: "Order ID",
  //     flex: 0.5,
  //   },
  //   {
  //     field: "productid",
  //     headerName: "Product ID",
  //     flex: 0.5,
  //   },
  //   {
  //     field: "quantity",
  //     headerName: "Quantity",
  //     flex: 0.5,
  //   },
  //   {
  //     field: "shelfid",
  //     headerName: "Shelf ID",
  //     flex: 0.4,
  //   },
  //   {
  //     field: "orderproductstatus",
  //     headerName: "Order-Product Status",
  //     flex: 0.4,
  //   },
  //   {
  //     field: "orderdate",
  //     headerName: "Order Date-Time",
  //     flex: 0.4,
  //   },
  // ];

  return (
    <Box m="1.5rem 2.5rem">
      <FlexBetween>
        <Header title="DASHBOARD" subtitle="Welcome to your dashboard" />

        <Box>
          <Button
            sx={{
              backgroundColor: theme.palette.secondary.light,
              color: theme.palette.background.alt,
              fontSize: "14px",
              fontWeight: "bold",
              padding: "10px 20px",
            }}
          >
            <DownloadOutlined sx={{ mr: "10px" }} />
            Download Reports
          </Button>
        </Box>
      </FlexBetween>

      <Box
        mt="20px"
        display="grid"
        gridTemplateColumns="repeat(12, 1fr)"
        gridAutoRows="160px"
        gap="20px"
        sx={{
          "& > div": { gridColumn: isNonMediumScreens ? undefined : "span 12" },
        }}
      >
        {/* ROW 1 */}
        <StatBox
          title="Total Customers"
          value={data && data.totalCustomers}
          increase="+14%"
          description="Since last month"
          icon={
            <Email
              sx={{ color: theme.palette.secondary[300], fontSize: "26px" }}
            />
          }
        />
        <StatBox
          title="Sales Today"
          value={data && data.todayStats.totalSales}
          increase="+21%"
          description="Since last day"
          icon={
            <PointOfSale
              sx={{ color: theme.palette.secondary[300], fontSize: "26px" }}
            />
          }
        />
        <Box
          gridColumn="span 8"
          gridRow="span 2"
          backgroundColor={theme.palette.background.alt}
          p="1rem"
          borderRadius="0.55rem"
        >
          <OverviewChart view="sales" isDashboard={true} />
        </Box>
        <StatBox
          title="Monthly Sales"
          value={data && data.thisMonthStats.totalSales}
          increase="+5%"
          description="Since last month"
          icon={
            <PersonAdd
              sx={{ color: theme.palette.secondary[300], fontSize: "26px" }}
            />
          }
        />
        <StatBox
          title="Yearly Sales"
          value={data && data.yearlySalesTotal}
          increase="+43%"
          description="Since last year"
          icon={
            <Traffic
              sx={{ color: theme.palette.secondary[300], fontSize: "26px" }}
            />
          }
        />

        {/* ROW 2 */}
        <Box
          gridColumn="span 8"
          gridRow="span 3"
          sx={{
            "& .MuiDataGrid-root": {
              border: "none",
              borderRadius: "5rem",
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
              backgroundColor: theme.palette.background.alt,
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
          {/* <div id ='dashboard_table' style={{ height: 350, width: '100%' }}> */}
          {/* <DataGrid 
            loading={isLoading2 || !data_orders}
            getRowId={(row : any) => row.orderid+row.productid}
            rows={data_orders || []}
            columns={columns}
          /> */}
          {/* </div> */}
        </Box>
        <Box
          gridColumn="span 4"
          gridRow="span 3"
          backgroundColor={theme.palette.background.alt}
          p="1.5rem"
          borderRadius="0.55rem"
        >
          <Typography variant="h6" sx={{ color: theme.palette.secondary[100] }}>
            Sales By Category
          </Typography>
          <BreakdownChart isDashboard={true} />
          <Typography
            p="0 0.6rem"
            fontSize="0.8rem"
            sx={{ color: theme.palette.secondary[200] }}
          >
            Breakdown of real states and information via category for revenue
            made for this year and total sales.
          </Typography>
        </Box>
      </Box>
    </Box>
  );
};

export default Dashboard;
