// Bewaarde Lichtblick-layouts wissen als de standaardlayout veranderd is
// (TB_LAYOUT in de Dockerfile). Staat Lichtblick nog open in een ander
// tabblad, dan wacht het wissen (en Lichtblick zelf) tot dat tabblad dicht
// is - Chrome meldt dat niet (geen 'blocked'). Na 3 s nog niet klaar: een
// balk tonen in plaats van een leeg scherm; de pagina laadt dan vanzelf.
try {
  if (localStorage.getItem("tbLayout") !== "__TB_LAYOUT__") {
    localStorage.removeItem("studio.layout");
    var done = false;
    var req = indexedDB.deleteDatabase("lichtblick-layouts");
    req.onsuccess = req.onerror = function () {
      done = true;
      localStorage.setItem("tbLayout", "__TB_LAYOUT__");
      var bar = document.getElementById("tb-layout-bar");
      if (bar) bar.remove();
    };
    setTimeout(function () {
      if (done || !document.body) return;
      var d = document.createElement("div");
      d.id = "tb-layout-bar";
      d.textContent = "Nieuwe Lichtblick-layout: sluit de andere tabbladen met localhost:8080 - deze pagina laadt dan vanzelf.";
      d.style.cssText = "position:fixed;top:0;left:0;right:0;z-index:2147483647;background:#ff9500;color:#000;padding:10px;font:600 15px sans-serif;text-align:center";
      document.body.appendChild(d);
    }, 3000);
  }
} catch (e) {}
