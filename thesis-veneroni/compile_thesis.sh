#!/bin/bash

# Script per compilazione LaTeX con gestione errori
# Uso: ./compile_thesis.sh

TEX_FILE="tesi_unimi_dobot_cr5.tex"
OUTPUT_NAME="tesi"

echo "🚀 Inizio compilazione tesi LaTeX..."

# Controllo esistenza file
if [ ! -f "$TEX_FILE" ]; then
    echo "❌ Errore: File $TEX_FILE non trovato!"
    exit 1
fi

echo "🛠️  Tentativo compilazione con latexmk + lualatex..."
if command -v latexmk >/dev/null 2>&1 && command -v lualatex >/dev/null 2>&1; then
    latexmk -lualatex -jobname="$OUTPUT_NAME" -interaction=nonstopmode -halt-on-error "$TEX_FILE" > compile.log 2>&1
    BUILD_STATUS=$?
else
    echo "ℹ️ latexmk o lualatex non disponibili, fallback a pdflatex multipass"
    pdflatex -jobname="$OUTPUT_NAME" -interaction=nonstopmode "$TEX_FILE" > compile.log 2>&1
    pdflatex -jobname="$OUTPUT_NAME" -interaction=nonstopmode "$TEX_FILE" >> compile.log 2>&1
    pdflatex -jobname="$OUTPUT_NAME" -interaction=nonstopmode "$TEX_FILE" >> compile.log 2>&1
    BUILD_STATUS=$?
fi

# Controllo risultato finale
if [ $BUILD_STATUS -eq 0 ] && [ -f "${OUTPUT_NAME}.pdf" ]; then
    echo ""
    echo "🎉 SUCCESSO! Tesi compilata con successo!"
    echo "📄 File generato: ${OUTPUT_NAME}.pdf"
    echo "📊 Dimensione: $(du -h "${OUTPUT_NAME}.pdf" | cut -f1)"
    echo "📝 Log di compilazione disponibile in: compile.log"
    
    # Mostra eventuali warning principali
    echo ""
    echo "📋 Riepilogo warnings principali:"
    grep -E "(Warning|Error|Overfull)" compile.log | head -10
    
    # Pulizia file temporanei (opzionale)
    read -p "🧹 Vuoi pulire i file temporanei? (y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm -f *.aux *.log *.toc *.lof *.lot *.lol *.out *.fdb_latexmk *.fls *.synctex.gz
        echo "✨ File temporanei rimossi"
    fi
    
else
    echo ""
    echo "❌ ERRORE: Compilazione fallita!"
    echo "📝 Controlla il file compile.log per dettagli:"
    echo ""
    tail -20 compile.log
    exit 1
fi

echo ""
echo "🎓 Tesi pronta per la consegna!"