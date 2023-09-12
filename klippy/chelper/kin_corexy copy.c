// CoreXY kinematics stepper pulse time generation
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdafx.h>
#include <iostream>
#include <fstream>
#include <iomanip>
using namespace std;
int main()
{
setlocale (LC_ALL, «RUS»);
int i, n;
double a;
ofstream f;
f.open(«C:\\sites\\accounts.txt», ios::out);
cout<<«n=»; cin>>n;
for (i=0; i<n; i++)
{
cout<<«a=»;
cin>>a;
f<<a<<«\t«;
}
f.close();
system(«pause»);
return 0;
}