set nocompatible
set ttyfast
set lazyredraw
set shiftwidth=2
set tabstop=2
set softtabstop=2
set expandtab
set hlsearch incsearch
set ignorecase smartcase
set infercase

" sensible.vim
set autoindent
set backspace=indent,eol,start
set complete-=i
set showmatch
set smarttab
set nrformats-=octal
set shiftround
set ttimeout
set ttimeoutlen=50
set incsearch
" Use <C-L> to clear the highlighting of :set hlsearch.
if maparg('<C-L>', 'n') ==# ''
  nnoremap <silent> <C-L> :nohlsearch<CR><C-L>
endif
set laststatus=1
set ruler
set showcmd
set wildmenu
if !&scrolloff
  set scrolloff=1
endif
if !&sidescrolloff
  set sidescrolloff=5
endif
set display+=lastline

if &encoding ==# 'latin1' && has('gui_running')
  set encoding=utf-8
endif

if &listchars ==# 'eol:$'
  set listchars=tab:>\ ,trail:-,extends:>,precedes:<,nbsp:+
  if &termencoding ==# 'utf-8' || &encoding ==# 'utf-8'
    let &listchars = "tab:\u21e5 ,trail:\u2423,extends:\u21c9,precedes:\u21c7,nbsp:\u00b7"
  endif
endif

"set rulerformat=%l/%L,%c%V%=%P
"set number
syntax enable
filetype plugin indent on
set cindent
set t_Co=256
set background=dark
colorscheme koehler
set spellfile=~/.vimspell.add
if has("autocmd")
  au BufReadPost * if line("'\"") > 0 && line("'\"") <= line("$")
    \| exe "normal! g'\"" | endif
  au FileType plaintex,tex set spell
endif

if has("gui_gtk2")
  set guifont=Monospace\ 11,Fixed\ 11
endif

" Remove spaces at end of lines
" http://stackoverflow.com/questions/356126/how-can-you-automatically-remove-trailing-whitespace-in-vim/1618401#1618401
fun! <SID>StripTrailingWhitespaces()
    let l = line(".")
    let c = col(".")
    %s/\s\+$//e
    call cursor(l, c)
endfun
if has("autocmd")
  au BufWritePre <buffer> :call <SID>StripTrailingWhitespaces()
endif

" Highlight spaces at the end of lines
match Todo /\s\+$/

" fold settings
set foldmethod=syntax
set foldnestmax=1
set nofoldenable

set linebreak

" Move through wrapped lines
map  <silent> <Up>   gk
imap <silent> <Up>   <C-o>gk
map  <silent> <Down> gj
imap <silent> <Down> <C-o>gj
map  <silent> <home> g<home>
imap <silent> <home> <C-o>g<home>
map  <silent> <End>  g<End>
imap <silent> <End>  <C-o>g<End>

" ROS launch files
autocmd BufRead,BufNewFile *.launch setfiletype xml
autocmd BufRead,BufNewFile *.machine setfiletype xml
