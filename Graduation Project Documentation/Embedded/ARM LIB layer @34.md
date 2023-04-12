> ## ***This is a documentation for issue #11***  
> `creating ARM libraries`  

## We have 2 libraries:
- STD-TYPES
- BIT-MATH  

## 1. STD-TYPES
This library uses the keyword `typedef` to define a new type of an old data type name.  

***Example:*** 
```C 
typedef unsigned short int u16;
```
`unsigned short int` is the old type, `u16` is the new type.

## 2. BIT-MATH
This library defines some math operations needed on registers such as:
- Set bit
- Clear bit
- Toggle bit
- Get bit