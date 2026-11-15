# 3D-modellen in htmlsim

Alle modellen zijn **CC0** (Creative Commons Zero, publiek domein): vrij te gebruiken, aan te passen en te verspreiden,
ook commercieel, zonder vermelding. We vermelden de bron toch.

| Pakket | Maker | Licentie | Gebruikt voor |
|---|---|---|---|
| Ultimate Modular Men/Women (via poly.pizza/u/Quaternius) | Quaternius (quaternius.com) | CC0 | `persoon_a` … `persoon_f`, `arbeider`, `boer`, `vrouw_jurk` |
| Car Kit | Kenney (kenney.nl) | CC0 | `auto_sedan`, `auto_suv`, `bestelwagen`, `pickup`, `kegel` |
| Furniture Kit 2.0 | Kenney (kenney.nl) | CC0 | stoelen, bureau, laptop, scherm, tv, zetel, boekenkast, plant, bed, toilet, ziekenhuisbed, nachtkastje, wastafel, wachtstoel, vuilnisbak, kapstok |
| Mini Characters | Kenney (kenney.nl) | CC0 | `rolstoel`, `defibrillator` (de personages zelf niet) |

Aangepast voor de simulator (`models.json`): op ware grootte geschaald, voorkant naar +x, midden op de oorsprong,
onderkant op 0, als één GLB per model. De personages staan in de idle-pose (animatie *Idle* uit de GLB), gebakken tot een vaste mesh, met echte
verhoudingen (hoofd ±1/8 van de lengte). Eerder Kenney Animated Characters: te groot hoofd (±1/4), vervangen.
Getest met YOLOX (COCO): personen als *person*, voertuigen als *car*/*truck*, de laptop en de bureaustoel meestal.
Poly Pizza-pagina's: Business Man JFrLIKqvCH, Worker Yg2bQZO6Hj, Farmer 7pn3R6hPvE, Casual Character kZ3DmIoGip,
Hoodie Character gKLBoRsyKe, Woman in Dress zMyPlQXBzq, Woman Casual jpKRgGDxhk, Woman AQsd9ngvKU,
Woman in Tank Top XqzeZGB7iU (poly.pizza/m/<id>, alle CC0).
