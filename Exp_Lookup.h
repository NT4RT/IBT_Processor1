// Lookup Table for converting log detector to linear power responce.
//  Bruce Randall  July 30, 2018, Sept18, 2018
// Table below from spreadsheet calcs.  16383 = Full Scale.
#define TableShift 6          // Shift right count for 255 full scale to PWM
#define TableLen 1025         // dummy on end for interp process
const unsigned int lookup[TableLen] PROGMEM = {
155,//
156,//
157,//
157,//
158,//
159,//
159,//
160,//
161,//
162,//
162,//
163,//
164,//
165,//
165,//
166,//
167,//
168,//
168,//
169,//
170,//
171,//
172,//
172,//
173,//
174,//
175,//
176,//
176,//
177,//
178,//
179,//
180,//
180,//
181,//
182,//
183,//
184,//
185,//
185,//
186,//
187,//
188,//
189,//
190,//
191,//
191,//
192,//
193,//
194,//
195,//
196,//
197,//
198,//
198,//
199,//
200,//
201,//
202,//
203,//
204,//
205,//
206,//
207,//
208,//
209,//
210,//
211,//
212,//
213,//
213,//
214,//
215,//
216,//
217,//
218,//
219,//
220,//
221,//
222,//
223,//
224,//
225,//
226,//
228,//
229,//
230,//
231,//
232,//
233,//
234,//
235,//
236,//
237,//
238,//
239,//
240,//
241,//
243,//
244,//
245,//
246,//
247,//
248,//
249,//
250,//
252,//
253,//
254,//
255,//
256,//
257,//
258,//
260,//
261,//
262,//
263,//
264,//
266,//
267,//
268,//
269,//
271,//
272,//
273,//
274,//
275,//
277,//
278,//
279,//
281,//
282,//
283,//
284,//
286,//
287,//
288,//
290,//
291,//
292,//
294,//
295,//
296,//
298,//
299,//
300,//
302,//
303,//
305,//
306,//
307,//
309,//
310,//
312,//
313,//
314,//
316,//
317,//
319,//
320,//
322,//
323,//
325,//
326,//
328,//
329,//
331,//
332,//
334,//
335,//
337,//
338,//
340,//
341,//
343,//
344,//
346,//
348,//
349,//
351,//
352,//
354,//
356,//
357,//
359,//
360,//
362,//
364,//
365,//
367,//
369,//
370,//
372,//
374,//
376,//
377,//
379,//
381,//
382,//
384,//
386,//
388,//
389,//
391,//
393,//
395,//
397,//
398,//
400,//
402,//
404,//
406,//
408,//
409,//
411,//
413,//
415,//
417,//
419,//
421,//
423,//
425,//
427,//
429,//
430,//
432,//
434,//
436,//
438,//
440,//
442,//
444,//
446,//
449,//
451,//
453,//
455,//
457,//
459,//
461,//
463,//
465,//
467,//
469,//
472,//
474,//
476,//
478,//
480,//
482,//
485,//
487,//
489,//
491,//
494,//
496,//
498,//
500,//
503,//
505,//
507,//
510,//
512,//
514,//
517,//
519,//
521,//
524,//
526,//
528,//
531,//
533,//
536,//
538,//
541,//
543,//
546,//
548,//
551,//
553,//
556,//
558,//
561,//
563,//
566,//
568,//
571,//
574,//
576,//
579,//
581,//
584,//
587,//
589,//
592,//
595,//
598,//
600,//
603,//
606,//
609,//
611,//
614,//
617,//
620,//
623,//
625,//
628,//
631,//
634,//
637,//
640,//
643,//
646,//
649,//
652,//
655,//
658,//
661,//
664,//
667,//
670,//
673,//
676,//
679,//
682,//
685,//
688,//
691,//
694,//
698,//
701,//
704,//
707,//
710,//
714,//
717,//
720,//
724,//
727,//
730,//
733,//
737,//
740,//
744,//
747,//
750,//
754,//
757,//
761,//
764,//
768,//
771,//
775,//
778,//
782,//
785,//
789,//
793,//
796,//
800,//
803,//
807,//
811,//
814,//
818,//
822,//
826,//
829,//
833,//
837,//
841,//
845,//
849,//
852,//
856,//
860,//
864,//
868,//
872,//
876,//
880,//
884,//
888,//
892,//
896,//
900,//
904,//
909,//
913,//
917,//
921,//
925,//
929,//
934,//
938,//
942,//
947,//
951,//
955,//
960,//
964,//
968,//
973,//
977,//
982,//
986,//
991,//
995,//
1000,//
1004,//
1009,//
1014,//
1018,//
1023,//
1027,//
1032,//
1037,//
1042,//
1046,//
1051,//
1056,//
1061,//
1066,//
1070,//
1075,//
1080,//
1085,//
1090,//
1095,//
1100,//
1105,//
1110,//
1115,//
1120,//
1125,//
1131,//
1136,//
1141,//
1146,//
1151,//
1157,//
1162,//
1167,//
1173,//
1178,//
1183,//
1189,//
1194,//
1200,//
1205,//
1211,//
1216,//
1222,//
1227,//
1233,//
1238,//
1244,//
1250,//
1255,//
1261,//
1267,//
1273,//
1279,//
1284,//
1290,//
1296,//
1302,//
1308,//
1314,//
1320,//
1326,//
1332,//
1338,//
1344,//
1350,//
1356,//
1363,//
1369,//
1375,//
1381,//
1388,//
1394,//
1400,//
1407,//
1413,//
1420,//
1426,//
1433,//
1439,//
1446,//
1452,//
1459,//
1466,//
1472,//
1479,//
1486,//
1493,//
1499,//
1506,//
1513,//
1520,//
1527,//
1534,//
1541,//
1548,//
1555,//
1562,//
1569,//
1576,//
1584,//
1591,//
1598,//
1605,//
1613,//
1620,//
1628,//
1635,//
1642,//
1650,//
1657,//
1665,//
1673,//
1680,//
1688,//
1696,//
1703,//
1711,//
1719,//
1727,//
1735,//
1743,//
1751,//
1759,//
1767,//
1775,//
1783,//
1791,//
1799,//
1807,//
1816,//
1824,//
1832,//
1841,//
1849,//
1857,//
1866,//
1874,//
1883,//
1892,//
1900,//
1909,//
1918,//
1926,//
1935,//
1944,//
1953,//
1962,//
1971,//
1980,//
1989,//
1998,//
2007,//
2016,//
2025,//
2035,//
2044,//
2053,//
2063,//
2072,//
2081,//
2091,//
2100,//
2110,//
2120,//
2129,//
2139,//
2149,//
2159,//
2168,//
2178,//
2188,//
2198,//
2208,//
2218,//
2229,//
2239,//
2249,//
2259,//
2270,//
2280,//
2290,//
2301,//
2311,//
2322,//
2332,//
2343,//
2354,//
2364,//
2375,//
2386,//
2397,//
2408,//
2419,//
2430,//
2441,//
2452,//
2463,//
2475,//
2486,//
2497,//
2509,//
2520,//
2532,//
2543,//
2555,//
2566,//
2578,//
2590,//
2602,//
2614,//
2626,//
2638,//
2650,//
2662,//
2674,//
2686,//
2698,//
2711,//
2723,//
2735,//
2748,//
2760,//
2773,//
2786,//
2798,//
2811,//
2824,//
2837,//
2850,//
2863,//
2876,//
2889,//
2902,//
2916,//
2929,//
2942,//
2956,//
2969,//
2983,//
2996,//
3010,//
3024,//
3038,//
3051,//
3065,//
3079,//
3093,//
3108,//
3122,//
3136,//
3150,//
3165,//
3179,//
3194,//
3208,//
3223,//
3238,//
3252,//
3267,//
3282,//
3297,//
3312,//
3327,//
3342,//
3358,//
3373,//
3388,//
3404,//
3419,//
3435,//
3451,//
3466,//
3482,//
3498,//
3514,//
3530,//
3546,//
3563,//
3579,//
3595,//
3612,//
3628,//
3645,//
3661,//
3678,//
3695,//
3712,//
3729,//
3746,//
3763,//
3780,//
3797,//
3814,//
3832,//
3849,//
3867,//
3885,//
3902,//
3920,//
3938,//
3956,//
3974,//
3992,//
4010,//
4029,//
4047,//
4066,//
4084,//
4103,//
4121,//
4140,//
4159,//
4178,//
4197,//
4216,//
4236,//
4255,//
4274,//
4294,//
4314,//
4333,//
4353,//
4373,//
4393,//
4413,//
4433,//
4453,//
4474,//
4494,//
4515,//
4535,//
4556,//
4577,//
4598,//
4619,//
4640,//
4661,//
4682,//
4703,//
4725,//
4746,//
4768,//
4790,//
4812,//
4834,//
4856,//
4878,//
4900,//
4923,//
4945,//
4968,//
4990,//
5013,//
5036,//
5059,//
5082,//
5105,//
5129,//
5152,//
5175,//
5199,//
5223,//
5247,//
5271,//
5295,//
5319,//
5343,//
5368,//
5392,//
5417,//
5441,//
5466,//
5491,//
5516,//
5541,//
5567,//
5592,//
5618,//
5643,//
5669,//
5695,//
5721,//
5747,//
5773,//
5800,//
5826,//
5853,//
5879,//
5906,//
5933,//
5960,//
5988,//
6015,//
6042,//
6070,//
6098,//
6125,//
6153,//
6181,//
6210,//
6238,//
6267,//
6295,//
6324,//
6353,//
6382,//
6411,//
6440,//
6470,//
6499,//
6529,//
6559,//
6588,//
6619,//
6649,//
6679,//
6710,//
6740,//
6771,//
6802,//
6833,//
6864,//
6895,//
6927,//
6959,//
6990,//
7022,//
7054,//
7087,//
7119,//
7151,//
7184,//
7217,//
7250,//
7283,//
7316,//
7349,//
7383,//
7417,//
7451,//
7485,//
7519,//
7553,//
7588,//
7622,//
7657,//
7692,//
7727,//
7762,//
7798,//
7833,//
7869,//
7905,//
7941,//
7977,//
8014,//
8050,//
8087,//
8124,//
8161,//
8198,//
8236,//
8273,//
8311,//
8349,//
8387,//
8426,//
8464,//
8503,//
8541,//
8580,//
8620,//
8659,//
8698,//
8738,//
8778,//
8818,//
8858,//
8899,//
8939,//
8980,//
9021,//
9062,//
9104,//
9145,//
9187,//
9229,//
9271,//
9314,//
9356,//
9399,//
9442,//
9485,//
9528,//
9572,//
9615,//
9659,//
9703,//
9748,//
9792,//
9837,//
9882,//
9927,//
9972,//
10018,//
10063,//
10109,//
10155,//
10202,//
10248,//
10295,//
10342,//
10389,//
10437,//
10484,//
10532,//
10580,//
10629,//
10677,//
10726,//
10775,//
10824,//
10873,//
10923,//
10973,//
11023,//
11073,//
11124,//
11175,//
11226,//
11277,//
11328,//
11380,//
11432,//
11484,//
11537,//
11589,//
11642,//
11695,//
11749,//
11802,//
11856,//
11910,//
11965,//
12019,//
12074,//
12129,//
12185,//
12240,//
12296,//
12352,//
12409,//
12465,//
12522,//
12579,//
12637,//
12695,//
12753,//
12811,//
12869,//
12928,//
12987,//
13046,//
13106,//
13166,//
13226,//
13286,//
13347,//
13408,//
13469,//
13530,//
13592,//
13654,//
13717,//
13779,//
13842,//
13905,//
13969,//
14032,//
14097,//
14161,//
14226,//
14290,//
14356,//
14421,//
14487,//
14553,//
14620,//
14686,//
14753,//
14821,//
14888,//
14956,//
15025,//
15093,//
15162,//
15231,//
15301,//
15371,//
15441,//
15511,//
15582,//
15653,//
15725,//
15797,//
15869,//
15941,//
16014,//
16087,//
16160,//
16234,//
16308,//
16383,//
16383 // dummy on end
};
