# 3D-modellen in htmlsim

Alle modellen zijn **CC0** (Creative Commons Zero, publiek domein): vrij te gebruiken, aan te passen en te verspreiden,
ook commercieel, zonder vermelding. We vermelden de bron toch.

| Pakket | Maker | Licentie | Gebruikt voor |
|---|---|---|---|
| Ultimate Modular Men/Women (via poly.pizza/u/Quaternius) | Quaternius (quaternius.com) | CC0 | `persoon_a` … `persoon_f`, `arbeider`, `boer`, `vrouw_jurk` |
| Poly Haven (polyhaven.com, 3D-scans) | Arrangemonk, Ethan Place, Garreth Dean, James Ray Cock, Kirill Sannikov, Ulan Cabanilla, Vibrant Nordic | CC0 | `stoel` (SchoolChair_01), `bureau` (metal_office_desk), `laptop` (classic_laptop), `zetel` (sofa_02), `boekenkast` (wooden_bookshelf_worn), `plant` (potted_plant_04), `nachtkastje` (ClassicNightstand_01), `rolstoel` (wheelchair_01), `wachtstoel` (modern_arm_chair_01), `plant_groot` (potted_plant_02) |
| Poly Pizza (poly.pizza) | Quaternius, CreativeTrio | CC0 | `auto_sedan` (Car Cz6yDaUcM9), `auto_suv` (SUV xsMtZhBkxL), `pickup` (Pickup Truck qn4grQgHm8), `bureaustoel` (Office Chair UfKvrZBK6C), `scherm`/`tv` (Monitor PvSjEbz11k), `toilet` (WAu50yGFVt), `wastafel` (Bathroom Sink OMCJDgMUui), `vuilnisbak` (Trashcan XSwahu252t), `kegel` (Traffic Cone aDIrUbMbW3), `kapstok` (Coat Rack uTZAW98YBS), `bed`/`ziekenhuisbed` (Bed Single ianC28eMOF), `boom_loof` (Tree qZtx0AHhcy), `boom_berk` (Birch Tree k6cozoNfLH), `boom_den` (Pine 79gmlLnweB), `gewas_mais` (Corn IhhB7kGaAQ) |
| Car Kit | Kenney (kenney.nl) | CC0 | `bestelwagen` |
| Mini Characters | Kenney (kenney.nl) | CC0 | `defibrillator` |

Aangepast voor de simulator (`models.json`): op ware grootte geschaald, voorkant naar +x, midden op de oorsprong,
onderkant op 0, als één GLB per model. Texturen: enkel de basiskleur, max. 512 px JPEG. Auto's en enkele meubels
per as geschaald naar gangbare maten (bv. bureau 1,4 × 0,75 m); de rolstoel vereenvoudigd (40k → ±10k driehoeken). De personages staan in de idle-pose (animatie *Idle* uit de GLB), gebakken tot een vaste mesh, met echte
verhoudingen (hoofd ±1/8 van de lengte). Eerder Kenney Animated Characters: te groot hoofd (±1/4), vervangen.
Getest met YOLOX (COCO): personen als *person*, voertuigen als *car*/*truck*, de laptop en de bureaustoel meestal.
Poly Pizza-pagina's: Business Man JFrLIKqvCH, Worker Yg2bQZO6Hj, Farmer 7pn3R6hPvE, Casual Character kZ3DmIoGip,
Hoodie Character gKLBoRsyKe, Woman in Dress zMyPlQXBzq, Woman Casual jpKRgGDxhk, Woman AQsd9ngvKU,
Woman in Tank Top XqzeZGB7iU (poly.pizza/m/<id>, alle CC0).
