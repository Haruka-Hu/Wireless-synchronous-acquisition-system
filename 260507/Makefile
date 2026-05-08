MAIN = main
NAME = ustcbeamer
STYFILES = $(NAME).sty

SHELL = bash
LATEXMK = latexmk -xelatex
XELATEX = xelatex -interaction=nonstopmode -halt-on-error

.PHONY : main sty clean all FORCE_MAKE

main : $(MAIN).pdf

all : main

sty : $(STYFILES)

$(MAIN).pdf : $(MAIN).tex $(STYFILES) FORCE_MAKE
	@if command -v latexmk >/dev/null 2>&1; then \
		$(LATEXMK) $<; \
	else \
		echo "latexmk not found, fallback to xelatex (2 passes)"; \
		$(XELATEX) $< && $(XELATEX) $<; \
	fi

clean : FORCE_MAKE
	@if command -v latexmk >/dev/null 2>&1; then \
		$(LATEXMK) -c $(MAIN).tex; \
	else \
		rm -f $(MAIN).aux $(MAIN).fdb_latexmk $(MAIN).fls $(MAIN).log $(MAIN).nav $(MAIN).out $(MAIN).snm $(MAIN).toc $(MAIN).xdv; \
	fi

distclean :
	@if command -v latexmk >/dev/null 2>&1; then \
		$(LATEXMK) -C $(MAIN).tex; \
	else \
		rm -f $(MAIN).aux $(MAIN).fdb_latexmk $(MAIN).fls $(MAIN).log $(MAIN).nav $(MAIN).out $(MAIN).pdf $(MAIN).snm $(MAIN).toc $(MAIN).xdv; \
	fi
