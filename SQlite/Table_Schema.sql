drop table if exists MFC;
drop table if exists GA;
drop table if exists ADAM_TC;
drop table if exists Scale;
drop table if exists DFM;


create table MFC (
Time REAL, 
Pressure REAL,
Temper REAL,
VolFlow REAL,
MassFlow REAL,
Setpoint REAL
);

create table GA (
Time REAL,
CO REAL,
CO2 REAL,
CH4 REAL,
H2 REAL,
N2 REAL,
HEAT REAL
);

create table Scale (
Time REAL,
Weight REAL
);

create table DFM (
Time REAL,
FlowRate REAL
);

create table ADAM_TC (
Time REAL,
TC_0 REAL,
TC_1 REAL,
TC_2 REAL,
TC_3 REAL,
TC_4 REAL,
TC_5 REAL,
TC_6 REAL,
TC_7 REAL
);