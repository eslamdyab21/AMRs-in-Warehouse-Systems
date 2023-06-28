import React, { useMemo, useState } from "react";
import { Box, useTheme } from "@mui/material";
import Header from "components/Header";
import { ResponsiveLine } from "@nivo/line";
import { useGetSalesQuery } from "state/api";
import DatePicker from "react-datepicker";
import "react-datepicker/dist/react-datepicker.css";

const dailyData_mock= [
  { date: "2021-01-02", totalSales: 4440, totalUnits: 178 },
  { date: "2021-01-03", totalSales: 9208, totalUnits: 277 },
  { date: "2021-01-04", totalSales: 4078, totalUnits: 564 },
  { date: "2021-01-05", totalSales: 7537, totalUnits: 521 },
  { date: "2021-01-06", totalSales: 2136, totalUnits: 871 },
  { date: "2021-01-07", totalSales: 6005, totalUnits: 503 },
  { date: "2021-01-08", totalSales: 8290, totalUnits: 604 },
  { date: "2021-01-09", totalSales: 5539, totalUnits: 167 },
  { date: "2021-01-10", totalSales: 2508, totalUnits: 769 },
  { date: "2021-01-11", totalSales: 7899, totalUnits: 220 },
  { date: "2021-01-12", totalSales: 7706, totalUnits: 793 },
  { date: "2021-01-13", totalSales: 9322, totalUnits: 528 },
  { date: "2021-01-14", totalSales: 4613, totalUnits: 889 },
  { date: "2021-01-15", totalSales: 6847, totalUnits: 898 },
  { date: "2021-01-16", totalSales: 5773, totalUnits: 505 },
  { date: "2021-01-17", totalSales: 7358, totalUnits: 796 },
  { date: "2021-01-18", totalSales: 3053, totalUnits: 535 },
  { date: "2021-01-19", totalSales: 6975, totalUnits: 137 },
  { date: "2021-01-20", totalSales: 5760, totalUnits: 567 },
  { date: "2021-01-21", totalSales: 1496, totalUnits: 651 },
  { date: "2021-01-22", totalSales: 4391, totalUnits: 742 },
  { date: "2021-01-23", totalSales: 3099, totalUnits: 687 },
  { date: "2021-01-24", totalSales: 3910, totalUnits: 845 },
  { date: "2021-01-25", totalSales: 7019, totalUnits: 286 },
  { date: "2021-01-26", totalSales: 7495, totalUnits: 687 },
  { date: "2021-01-27", totalSales: 3351, totalUnits: 561 },
  { date: "2021-01-28", totalSales: 8893, totalUnits: 837 },
  { date: "2021-01-29", totalSales: 2463, totalUnits: 347 },
  { date: "2021-01-30", totalSales: 5740, totalUnits: 973 },
  { date: "2021-01-31", totalSales: 2053, totalUnits: 758 },
  { date: "2021-02-01", totalSales: 6121, totalUnits: 864 },
  { date: "2021-02-02", totalSales: 8806, totalUnits: 776 },
  { date: "2021-02-03", totalSales: 1748, totalUnits: 624 },
  { date: "2021-02-04", totalSales: 9853, totalUnits: 468 },
  { date: "2021-02-05", totalSales: 4183, totalUnits: 434 },
  { date: "2021-02-06", totalSales: 5302, totalUnits: 882 },
  { date: "2021-02-07", totalSales: 3788, totalUnits: 722 },
  { date: "2021-02-08", totalSales: 7761, totalUnits: 451 },
  { date: "2021-02-09", totalSales: 3720, totalUnits: 659 },
  { date: "2021-02-10", totalSales: 5499, totalUnits: 825 },
  { date: "2021-02-11", totalSales: 3474, totalUnits: 180 },
  { date: "2021-02-12", totalSales: 2857, totalUnits: 687 },
  { date: "2021-02-13", totalSales: 2929, totalUnits: 736 },
  { date: "2021-02-14", totalSales: 7519, totalUnits: 690 },
  { date: "2021-02-15", totalSales: 7673, totalUnits: 233 },
  { date: "2021-02-16", totalSales: 6933, totalUnits: 330 },
  { date: "2021-02-17", totalSales: 4510, totalUnits: 837 },
  { date: "2021-02-18", totalSales: 4773, totalUnits: 847 },
  { date: "2021-02-19", totalSales: 8331, totalUnits: 213 },
  { date: "2021-02-20", totalSales: 1460, totalUnits: 975 },
  { date: "2021-02-21", totalSales: 2019, totalUnits: 954 },
  { date: "2021-02-22", totalSales: 4342, totalUnits: 608 },
  { date: "2021-02-23", totalSales: 4246, totalUnits: 865 },
  { date: "2021-02-24", totalSales: 4534, totalUnits: 873 },
  { date: "2021-02-25", totalSales: 3458, totalUnits: 428 },
  { date: "2021-02-26", totalSales: 9667, totalUnits: 843 },
  { date: "2021-02-27", totalSales: 4472, totalUnits: 469 },
  { date: "2021-02-28", totalSales: 3017, totalUnits: 922 },
  { date: "2021-02-29", totalSales: 2851, totalUnits: 802 },
  { date: "2021-03-01", totalSales: 8296, totalUnits: 768 },
  { date: "2021-03-02", totalSales: 5479, totalUnits: 340 },
  { date: "2021-03-03", totalSales: 9791, totalUnits: 712 },
  { date: "2021-03-04", totalSales: 7183, totalUnits: 174 },
  { date: "2021-03-05", totalSales: 8617, totalUnits: 240 },
  { date: "2021-03-06", totalSales: 4289, totalUnits: 980 },
  { date: "2021-03-07", totalSales: 4903, totalUnits: 140 },
  { date: "2021-03-08", totalSales: 4580, totalUnits: 782 },
  { date: "2021-03-09", totalSales: 7680, totalUnits: 952 },
  { date: "2021-03-10", totalSales: 8158, totalUnits: 514 },
  { date: "2021-03-11", totalSales: 1568, totalUnits: 173 },
  { date: "2021-03-12", totalSales: 7984, totalUnits: 961 },
  { date: "2021-03-13", totalSales: 4534, totalUnits: 765 },
  { date: "2021-03-14", totalSales: 6478, totalUnits: 777 },
  { date: "2021-03-15", totalSales: 7361, totalUnits: 126 },
  { date: "2021-03-16", totalSales: 8626, totalUnits: 406 },
  { date: "2021-03-17", totalSales: 6885, totalUnits: 834 },
  { date: "2021-03-18", totalSales: 7998, totalUnits: 194 },
  { date: "2021-03-19", totalSales: 1798, totalUnits: 133 },
  { date: "2021-03-20", totalSales: 1186, totalUnits: 392 },
  { date: "2021-03-21", totalSales: 7765, totalUnits: 355 },
  { date: "2021-03-22", totalSales: 6411, totalUnits: 561 },
  { date: "2021-03-23", totalSales: 6003, totalUnits: 434 },
  { date: "2021-03-24", totalSales: 9039, totalUnits: 881 },
  { date: "2021-03-25", totalSales: 4284, totalUnits: 260 },
  { date: "2021-03-26", totalSales: 9655, totalUnits: 823 },
  { date: "2021-03-27", totalSales: 3022, totalUnits: 747 },
  { date: "2021-03-28", totalSales: 5342, totalUnits: 546 },
  { date: "2021-03-29", totalSales: 9784, totalUnits: 234 },
  { date: "2021-03-30", totalSales: 5045, totalUnits: 301 },
  { date: "2021-03-31", totalSales: 5802, totalUnits: 429 },
  { date: "2021-04-01", totalSales: 4725, totalUnits: 649 },
  { date: "2021-04-02", totalSales: 8225, totalUnits: 673 },
  { date: "2021-04-03", totalSales: 3355, totalUnits: 995 },
  { date: "2021-04-04", totalSales: 8735, totalUnits: 539 },
  { date: "2021-04-05", totalSales: 8041, totalUnits: 433 },
  { date: "2021-04-06", totalSales: 6195, totalUnits: 900 },
  { date: "2021-04-07", totalSales: 5474, totalUnits: 562 },
  { date: "2021-04-08", totalSales: 6637, totalUnits: 922 },
  { date: "2021-04-09", totalSales: 7757, totalUnits: 260 },
  { date: "2021-04-10", totalSales: 1768, totalUnits: 888 },
  { date: "2021-04-11", totalSales: 6799, totalUnits: 394 },
  { date: "2021-04-12", totalSales: 1277, totalUnits: 387 },
  { date: "2021-04-13", totalSales: 6292, totalUnits: 932 },
  { date: "2021-04-14", totalSales: 1990, totalUnits: 174 },
  { date: "2021-04-15", totalSales: 6132, totalUnits: 870 },
  { date: "2021-04-16", totalSales: 9385, totalUnits: 503 },
  { date: "2021-04-17", totalSales: 8463, totalUnits: 744 },
  { date: "2021-04-18", totalSales: 5406, totalUnits: 719 },
  { date: "2021-04-19", totalSales: 7544, totalUnits: 508 },
  { date: "2021-04-20", totalSales: 8510, totalUnits: 992 },
  { date: "2021-04-21", totalSales: 7671, totalUnits: 507 },
  { date: "2021-04-22", totalSales: 5105, totalUnits: 291 },
  { date: "2021-04-23", totalSales: 9563, totalUnits: 621 },
  { date: "2021-04-24", totalSales: 6764, totalUnits: 627 },
  { date: "2021-04-25", totalSales: 2359, totalUnits: 909 },
  { date: "2021-04-26", totalSales: 5369, totalUnits: 329 },
  { date: "2021-04-27", totalSales: 3370, totalUnits: 827 },
  { date: "2021-04-28", totalSales: 2470, totalUnits: 274 },
  { date: "2021-04-29", totalSales: 5255, totalUnits: 846 },
  { date: "2021-04-30", totalSales: 2727, totalUnits: 878 },
  { date: "2021-05-01", totalSales: 1607, totalUnits: 311 },
  { date: "2021-05-02", totalSales: 6151, totalUnits: 561 },
  { date: "2021-05-03", totalSales: 2484, totalUnits: 465 },
  { date: "2021-05-04", totalSales: 9561, totalUnits: 179 },
  { date: "2021-05-05", totalSales: 2279, totalUnits: 820 },
  { date: "2021-05-06", totalSales: 9042, totalUnits: 545 },
  { date: "2021-05-07", totalSales: 1861, totalUnits: 731 },
  { date: "2021-05-08", totalSales: 3152, totalUnits: 554 },
  { date: "2021-05-09", totalSales: 4039, totalUnits: 323 },
  { date: "2021-05-10", totalSales: 6632, totalUnits: 314 },
  { date: "2021-05-11", totalSales: 4559, totalUnits: 909 },
  { date: "2021-05-12", totalSales: 1401, totalUnits: 125 },
  { date: "2021-05-13", totalSales: 6642, totalUnits: 460 },
  { date: "2021-05-14", totalSales: 7035, totalUnits: 168 },
  { date: "2021-05-15", totalSales: 3370, totalUnits: 401 },
  { date: "2021-05-16", totalSales: 8193, totalUnits: 935 },
  { date: "2021-05-17", totalSales: 2015, totalUnits: 120 },
  { date: "2021-05-18", totalSales: 6924, totalUnits: 844 },
  { date: "2021-05-19", totalSales: 8985, totalUnits: 781 },
  { date: "2021-05-20", totalSales: 4003, totalUnits: 457 },
  { date: "2021-05-21", totalSales: 8490, totalUnits: 125 },
  { date: "2021-05-22", totalSales: 1939, totalUnits: 114 },
  { date: "2021-05-23", totalSales: 9219, totalUnits: 131 },
  { date: "2021-05-24", totalSales: 8730, totalUnits: 734 },
  { date: "2021-05-25", totalSales: 1849, totalUnits: 629 },
  { date: "2021-05-26", totalSales: 7054, totalUnits: 206 },
  { date: "2021-05-27", totalSales: 9346, totalUnits: 749 },
  { date: "2021-05-28", totalSales: 7864, totalUnits: 776 },
  { date: "2021-05-29", totalSales: 5484, totalUnits: 326 },
  { date: "2021-05-30", totalSales: 4803, totalUnits: 429 },
  { date: "2021-05-31", totalSales: 7118, totalUnits: 279 },
  { date: "2021-06-01", totalSales: 1357, totalUnits: 588 },
  { date: "2021-06-02", totalSales: 5236, totalUnits: 943 },
  { date: "2021-06-03", totalSales: 9715, totalUnits: 380 },
  { date: "2021-06-04", totalSales: 4121, totalUnits: 437 },
  { date: "2021-06-05", totalSales: 9768, totalUnits: 698 },
  { date: "2021-06-06", totalSales: 8486, totalUnits: 351 },
  { date: "2021-06-07", totalSales: 1291, totalUnits: 368 },
  { date: "2021-06-08", totalSales: 2278, totalUnits: 453 },
  { date: "2021-06-09", totalSales: 5051, totalUnits: 174 },
  { date: "2021-06-10", totalSales: 6861, totalUnits: 863 },
  { date: "2021-06-11", totalSales: 2331, totalUnits: 169 },
  { date: "2021-06-12", totalSales: 8200, totalUnits: 655 },
  { date: "2021-06-13", totalSales: 9906, totalUnits: 194 },
  { date: "2021-06-14", totalSales: 9835, totalUnits: 343 },
  { date: "2021-06-15", totalSales: 8508, totalUnits: 592 },
  { date: "2021-06-16", totalSales: 9313, totalUnits: 1000 },
  { date: "2021-06-17", totalSales: 3612, totalUnits: 612 },
  { date: "2021-06-18", totalSales: 7732, totalUnits: 227 },
  { date: "2021-06-19", totalSales: 9594, totalUnits: 374 },
  { date: "2021-06-20", totalSales: 4254, totalUnits: 553 },
  { date: "2021-06-21", totalSales: 9011, totalUnits: 368 },
  { date: "2021-06-22", totalSales: 3390, totalUnits: 657 },
  { date: "2021-06-23", totalSales: 6151, totalUnits: 925 },
  { date: "2021-06-24", totalSales: 8028, totalUnits: 703 },
  { date: "2021-06-25", totalSales: 3258, totalUnits: 698 },
  { date: "2021-06-26", totalSales: 5536, totalUnits: 165 },
  { date: "2021-06-27", totalSales: 9745, totalUnits: 667 },
  { date: "2021-06-28", totalSales: 1066, totalUnits: 845 },
  { date: "2021-06-29", totalSales: 3550, totalUnits: 573 },
  { date: "2021-06-30", totalSales: 3656, totalUnits: 264 },
  { date: "2021-07-01", totalSales: 6496, totalUnits: 562 },
  { date: "2021-07-02", totalSales: 9990, totalUnits: 412 },
  { date: "2021-07-03", totalSales: 5603, totalUnits: 753 },
  { date: "2021-07-04", totalSales: 1077, totalUnits: 629 },
  { date: "2021-07-05", totalSales: 4093, totalUnits: 604 },
  { date: "2021-07-06", totalSales: 5120, totalUnits: 190 },
  { date: "2021-07-07", totalSales: 4515, totalUnits: 907 },
  { date: "2021-07-08", totalSales: 9038, totalUnits: 148 },
  { date: "2021-07-09", totalSales: 6667, totalUnits: 632 },
  { date: "2021-07-10", totalSales: 8122, totalUnits: 127 },
  { date: "2021-07-11", totalSales: 1972, totalUnits: 117 },
  { date: "2021-07-12", totalSales: 6801, totalUnits: 563 },
  { date: "2021-07-13", totalSales: 4091, totalUnits: 637 },
  { date: "2021-07-14", totalSales: 3113, totalUnits: 410 },
  { date: "2021-07-15", totalSales: 4585, totalUnits: 375 },
  { date: "2021-07-16", totalSales: 6517, totalUnits: 446 },
  { date: "2021-07-17", totalSales: 1881, totalUnits: 319 },
  { date: "2021-07-18", totalSales: 7364, totalUnits: 170 },
  { date: "2021-07-19", totalSales: 6019, totalUnits: 761 },
  { date: "2021-07-20", totalSales: 4955, totalUnits: 896 },
  { date: "2021-07-21", totalSales: 4778, totalUnits: 128 },
  { date: "2021-07-22", totalSales: 9728, totalUnits: 292 },
  { date: "2021-07-23", totalSales: 7928, totalUnits: 539 },
  { date: "2021-07-24", totalSales: 4345, totalUnits: 595 },
  { date: "2021-07-25", totalSales: 8472, totalUnits: 928 },
  { date: "2021-07-26", totalSales: 5393, totalUnits: 958 },
  { date: "2021-07-27", totalSales: 6508, totalUnits: 691 },
  { date: "2021-07-28", totalSales: 8991, totalUnits: 513 },
  { date: "2021-07-29", totalSales: 3115, totalUnits: 830 },
  { date: "2021-07-30", totalSales: 8772, totalUnits: 556 },
  { date: "2021-07-31", totalSales: 4551, totalUnits: 284 },
  { date: "2021-08-01", totalSales: 6332, totalUnits: 292 },
  { date: "2021-08-02", totalSales: 2518, totalUnits: 969 },
  { date: "2021-08-03", totalSales: 7598, totalUnits: 786 },
  { date: "2021-08-04", totalSales: 5509, totalUnits: 146 },
  { date: "2021-08-05", totalSales: 7491, totalUnits: 304 },
  { date: "2021-08-06", totalSales: 3664, totalUnits: 425 },
  { date: "2021-08-07", totalSales: 7173, totalUnits: 268 },
  { date: "2021-08-08", totalSales: 4771, totalUnits: 888 },
  { date: "2021-08-09", totalSales: 2593, totalUnits: 565 },
  { date: "2021-08-10", totalSales: 4951, totalUnits: 552 },
  { date: "2021-08-11", totalSales: 7876, totalUnits: 651 },
  { date: "2021-08-12", totalSales: 7589, totalUnits: 555 },
  { date: "2021-08-13", totalSales: 6829, totalUnits: 520 },
  { date: "2021-08-14", totalSales: 8193, totalUnits: 232 },
  { date: "2021-08-15", totalSales: 1075, totalUnits: 833 },
  { date: "2021-08-16", totalSales: 1384, totalUnits: 299 },
  { date: "2021-08-17", totalSales: 4635, totalUnits: 820 },
  { date: "2021-08-18", totalSales: 1886, totalUnits: 397 },
  { date: "2021-08-19", totalSales: 1461, totalUnits: 302 },
  { date: "2021-08-20", totalSales: 3612, totalUnits: 122 },
  { date: "2021-08-21", totalSales: 7275, totalUnits: 697 },
  { date: "2021-08-22", totalSales: 2319, totalUnits: 768 },
  { date: "2021-08-23", totalSales: 7268, totalUnits: 531 },
  { date: "2021-08-24", totalSales: 4286, totalUnits: 222 },
  { date: "2021-08-25", totalSales: 8294, totalUnits: 413 },
  { date: "2021-08-26", totalSales: 2961, totalUnits: 469 },
  { date: "2021-08-27", totalSales: 2920, totalUnits: 374 },
  { date: "2021-08-28", totalSales: 8327, totalUnits: 522 },
  { date: "2021-08-29", totalSales: 4722, totalUnits: 958 },
  { date: "2021-08-30", totalSales: 3219, totalUnits: 787 },
  { date: "2021-08-31", totalSales: 3713, totalUnits: 356 },
  { date: "2021-09-01", totalSales: 9797, totalUnits: 519 },
  { date: "2021-09-02", totalSales: 4632, totalUnits: 865 },
  { date: "2021-09-03", totalSales: 5273, totalUnits: 272 },
  { date: "2021-09-04", totalSales: 4765, totalUnits: 750 },
  { date: "2021-09-05", totalSales: 3041, totalUnits: 306 },
  { date: "2021-09-06", totalSales: 8139, totalUnits: 302 },
  { date: "2021-09-07", totalSales: 3348, totalUnits: 776 },
  { date: "2021-09-08", totalSales: 2433, totalUnits: 432 },
  { date: "2021-09-09", totalSales: 5919, totalUnits: 518 },
  { date: "2021-09-10", totalSales: 6850, totalUnits: 292 },
  { date: "2021-09-11", totalSales: 1371, totalUnits: 429 },
  { date: "2021-09-12", totalSales: 4895, totalUnits: 729 },
  { date: "2021-09-13", totalSales: 2404, totalUnits: 564 },
  { date: "2021-09-14", totalSales: 8578, totalUnits: 382 },
  { date: "2021-09-15", totalSales: 7339, totalUnits: 826 },
  { date: "2021-09-16", totalSales: 9930, totalUnits: 140 },
  { date: "2021-09-17", totalSales: 4042, totalUnits: 972 },
  { date: "2021-09-18", totalSales: 7080, totalUnits: 502 },
  { date: "2021-09-19", totalSales: 2858, totalUnits: 722 },
  { date: "2021-09-20", totalSales: 8279, totalUnits: 119 },
  { date: "2021-09-21", totalSales: 3473, totalUnits: 290 },
  { date: "2021-09-22", totalSales: 2254, totalUnits: 112 },
  { date: "2021-09-23", totalSales: 8813, totalUnits: 658 },
  { date: "2021-09-24", totalSales: 5857, totalUnits: 617 },
  { date: "2021-09-25", totalSales: 8542, totalUnits: 531 },
  { date: "2021-09-26", totalSales: 1537, totalUnits: 975 },
  { date: "2021-09-27", totalSales: 9325, totalUnits: 323 },
  { date: "2021-09-28", totalSales: 4286, totalUnits: 599 },
  { date: "2021-09-29", totalSales: 2375, totalUnits: 760 },
  { date: "2021-09-30", totalSales: 7925, totalUnits: 863 },
  { date: "2021-10-01", totalSales: 1766, totalUnits: 601 },
  { date: "2021-10-02", totalSales: 2578, totalUnits: 195 },
  { date: "2021-10-03", totalSales: 6840, totalUnits: 887 },
  { date: "2021-10-04", totalSales: 2257, totalUnits: 845 },
  { date: "2021-10-05", totalSales: 2168, totalUnits: 606 },
  { date: "2021-10-06", totalSales: 8791, totalUnits: 275 },
  { date: "2021-10-07", totalSales: 2005, totalUnits: 652 },
  { date: "2021-10-08", totalSales: 4086, totalUnits: 925 },
  { date: "2021-10-09", totalSales: 5851, totalUnits: 921 },
  { date: "2021-10-10", totalSales: 4390, totalUnits: 490 },
  { date: "2021-10-11", totalSales: 7271, totalUnits: 131 },
  { date: "2021-10-12", totalSales: 6548, totalUnits: 337 },
  { date: "2021-10-13", totalSales: 1736, totalUnits: 822 },
  { date: "2021-10-14", totalSales: 3722, totalUnits: 305 },
  { date: "2021-10-15", totalSales: 3772, totalUnits: 814 },
  { date: "2021-10-16", totalSales: 3169, totalUnits: 712 },
  { date: "2021-10-17", totalSales: 7579, totalUnits: 676 },
  { date: "2021-10-18", totalSales: 8615, totalUnits: 644 },
  { date: "2021-10-19", totalSales: 3766, totalUnits: 172 },
  { date: "2021-10-20", totalSales: 5803, totalUnits: 405 },
  { date: "2021-10-21", totalSales: 8430, totalUnits: 338 },
  { date: "2021-10-22", totalSales: 7648, totalUnits: 631 },
  { date: "2021-10-23", totalSales: 2434, totalUnits: 461 },
  { date: "2021-10-24", totalSales: 9520, totalUnits: 846 },
  { date: "2021-10-25", totalSales: 6588, totalUnits: 674 },
  { date: "2021-10-26", totalSales: 7180, totalUnits: 276 },
  { date: "2021-10-27", totalSales: 1039, totalUnits: 899 },
  { date: "2021-10-28", totalSales: 7440, totalUnits: 469 },
  { date: "2021-10-29", totalSales: 3026, totalUnits: 620 },
  { date: "2021-10-30", totalSales: 3594, totalUnits: 368 },
  { date: "2021-10-31", totalSales: 2710, totalUnits: 213 },
  { date: "2021-11-01", totalSales: 4998, totalUnits: 826 },
  { date: "2021-11-02", totalSales: 1856, totalUnits: 598 },
  { date: "2021-11-03", totalSales: 3032, totalUnits: 694 },
  { date: "2021-11-04", totalSales: 8275, totalUnits: 491 },
  { date: "2021-11-05", totalSales: 8105, totalUnits: 911 },
  { date: "2021-11-06", totalSales: 2815, totalUnits: 222 },
  { date: "2021-11-07", totalSales: 2183, totalUnits: 402 },
  { date: "2021-11-08", totalSales: 5988, totalUnits: 597 },
  { date: "2021-11-09", totalSales: 9263, totalUnits: 609 },
  { date: "2021-11-10", totalSales: 5886, totalUnits: 187 },
  { date: "2021-11-11", totalSales: 3003, totalUnits: 665 },
  { date: "2021-11-12", totalSales: 3093, totalUnits: 116 },
  { date: "2021-11-13", totalSales: 1791, totalUnits: 758 },
  { date: "2021-11-14", totalSales: 1540, totalUnits: 327 },
  { date: "2021-11-15", totalSales: 7916, totalUnits: 953 },
  { date: "2021-11-16", totalSales: 5222, totalUnits: 563 },
  { date: "2021-11-17", totalSales: 4137, totalUnits: 567 },
  { date: "2021-11-18", totalSales: 6127, totalUnits: 594 },
  { date: "2021-11-19", totalSales: 6733, totalUnits: 843 },
  { date: "2021-11-20", totalSales: 8355, totalUnits: 536 },
  { date: "2021-11-21", totalSales: 7293, totalUnits: 645 },
  { date: "2021-11-22", totalSales: 2042, totalUnits: 405 },
  { date: "2021-11-23", totalSales: 5261, totalUnits: 246 },
  { date: "2021-11-24", totalSales: 6218, totalUnits: 359 },
  { date: "2021-11-25", totalSales: 4316, totalUnits: 572 },
  { date: "2021-11-26", totalSales: 4855, totalUnits: 922 },
  { date: "2021-11-27", totalSales: 4620, totalUnits: 870 },
  { date: "2021-11-28", totalSales: 5238, totalUnits: 620 },
  { date: "2021-11-29", totalSales: 2176, totalUnits: 474 },
  { date: "2021-11-30", totalSales: 8352, totalUnits: 705 },
  { date: "2021-12-01", totalSales: 6264, totalUnits: 301 },
  { date: "2021-12-02", totalSales: 8046, totalUnits: 533 },
  { date: "2021-12-03", totalSales: 4552, totalUnits: 430 },
  { date: "2021-12-04", totalSales: 8425, totalUnits: 634 },
  { date: "2021-12-05", totalSales: 2423, totalUnits: 520 },
  { date: "2021-12-06", totalSales: 9756, totalUnits: 691 },
  { date: "2021-12-07", totalSales: 2974, totalUnits: 740 },
  { date: "2021-12-08", totalSales: 8947, totalUnits: 933 },
  { date: "2021-12-09", totalSales: 8435, totalUnits: 997 },
  { date: "2021-12-10", totalSales: 7246, totalUnits: 490 },
  { date: "2021-12-11", totalSales: 1691, totalUnits: 758 },
  { date: "2021-12-12", totalSales: 6938, totalUnits: 272 },
  { date: "2021-12-13", totalSales: 1443, totalUnits: 821 },
  { date: "2021-12-14", totalSales: 5855, totalUnits: 179 },
  { date: "2021-12-15", totalSales: 7722, totalUnits: 545 },
  { date: "2021-12-16", totalSales: 7117, totalUnits: 121 },
  { date: "2021-12-17", totalSales: 5420, totalUnits: 684 },
  { date: "2021-12-18", totalSales: 1271, totalUnits: 113 },
  { date: "2021-12-19", totalSales: 2586, totalUnits: 379 },
  { date: "2021-12-20", totalSales: 5116, totalUnits: 958 },
  { date: "2021-12-21", totalSales: 5393, totalUnits: 818 },
  { date: "2021-12-22", totalSales: 2832, totalUnits: 247 },
  { date: "2021-12-23", totalSales: 4626, totalUnits: 268 },
  { date: "2021-12-24", totalSales: 6757, totalUnits: 169 },
  { date: "2021-12-25", totalSales: 7515, totalUnits: 550 },
  { date: "2021-12-26", totalSales: 8269, totalUnits: 566 },
  { date: "2021-12-27", totalSales: 1322, totalUnits: 791 },
  { date: "2021-12-28", totalSales: 5404, totalUnits: 167 },
  { date: "2021-12-29", totalSales: 8295, totalUnits: 852 },
  { date: "2021-12-30", totalSales: 2525, totalUnits: 858 },
  { date: "2021-12-31", totalSales: 1326, totalUnits: 360 }
]

const Daily = () => {
  const [startDate, setStartDate] = useState(new Date("2021-02-01"));
  const [endDate, setEndDate] = useState(new Date("2021-03-01"));
  // const { data } = useGetSalesQuery();
  const data = dailyData_mock;
  const theme = useTheme();

  const [formattedData] = useMemo(() => {
    if (!data) return [];

    // const { dailyData } = data;
    const dailyData  = data
    const totalSalesLine = {
      id: "totalSales",
      color: theme.palette.secondary.main,
      data: [],
    };
    const totalUnitsLine = {
      id: "totalUnits",
      color: theme.palette.secondary[600],
      data: [],
    };

    Object.values(dailyData).forEach(({ date, totalSales, totalUnits }) => {
      const dateFormatted = new Date(date);
      if (dateFormatted >= startDate && dateFormatted <= endDate) {
        const splitDate = date.substring(date.indexOf("-") + 1);

        totalSalesLine.data = [
          ...totalSalesLine.data,
          { x: splitDate, y: totalSales },
        ];
        totalUnitsLine.data = [
          ...totalUnitsLine.data,
          { x: splitDate, y: totalUnits },
        ];
      }
    });

    const formattedData = [totalSalesLine, totalUnitsLine];
    return [formattedData];
  }, [data, startDate, endDate]); // eslint-disable-line react-hooks/exhaustive-deps

  return (
    <Box m="1.5rem 2.5rem">
      <Header title="DAILY SALES" subtitle="Chart of daily sales" />
      <Box height="75vh">
        <Box display="flex" justifyContent="flex-end">
          <Box>
            <DatePicker
              selected={startDate}
              onChange={(date) => setStartDate(date)}
              selectsStart
              startDate={startDate}
              endDate={endDate}
            />
          </Box>
          <Box>
            <DatePicker
              selected={endDate}
              onChange={(date) => setEndDate(date)}
              selectsEnd
              startDate={startDate}
              endDate={endDate}
              minDate={startDate}
            />
          </Box>
        </Box>

        {data ? (
          <ResponsiveLine
            data={formattedData}
            theme={{
              axis: {
                domain: {
                  line: {
                    stroke: theme.palette.secondary[200],
                  },
                },
                legend: {
                  text: {
                    fill: theme.palette.secondary[200],
                  },
                },
                ticks: {
                  line: {
                    stroke: theme.palette.secondary[200],
                    strokeWidth: 1,
                  },
                  text: {
                    fill: theme.palette.secondary[200],
                  },
                },
              },
              legends: {
                text: {
                  fill: theme.palette.secondary[200],
                },
              },
              tooltip: {
                container: {
                  color: theme.palette.primary.main,
                },
              },
            }}
            colors={{ datum: "color" }}
            margin={{ top: 50, right: 50, bottom: 70, left: 60 }}
            xScale={{ type: "point" }}
            yScale={{
              type: "linear",
              min: "auto",
              max: "auto",
              stacked: false,
              reverse: false,
            }}
            yFormat=" >-.2f"
            curve="catmullRom"
            axisTop={null}
            axisRight={null}
            axisBottom={{
              orient: "bottom",
              tickSize: 5,
              tickPadding: 5,
              tickRotation: 90,
              legend: "Month",
              legendOffset: 60,
              legendPosition: "middle",
            }}
            axisLeft={{
              orient: "left",
              tickSize: 5,
              tickPadding: 5,
              tickRotation: 0,
              legend: "Total",
              legendOffset: -50,
              legendPosition: "middle",
            }}
            enableGridX={false}
            enableGridY={false}
            pointSize={10}
            pointColor={{ theme: "background" }}
            pointBorderWidth={2}
            pointBorderColor={{ from: "serieColor" }}
            pointLabelYOffset={-12}
            useMesh={true}
            legends={[
              {
                anchor: "top-right",
                direction: "column",
                justify: false,
                translateX: 50,
                translateY: 0,
                itemsSpacing: 0,
                itemDirection: "left-to-right",
                itemWidth: 80,
                itemHeight: 20,
                itemOpacity: 0.75,
                symbolSize: 12,
                symbolShape: "circle",
                symbolBorderColor: "rgba(0, 0, 0, .5)",
                effects: [
                  {
                    on: "hover",
                    style: {
                      itemBackground: "rgba(0, 0, 0, .03)",
                      itemOpacity: 1,
                    },
                  },
                ],
              },
            ]}
          />
        ) : (
          <>Loading...</>
        )}
      </Box>
    </Box>
  );
};

export default Daily;
