set nocompatible
filetype off

" Vundle
" set the runtime path to include Vundle and initialize
set rtp+=~/.vim/bundle/Vundle.vim
call vundle#begin()

Plugin 'gmarik/Vundle.vim'
" Plugin 'godlygeek/csapprox'
Plugin 'antoyo/vim-licenses'
Plugin 'Valloric/YouCompleteMe'
" Plugin 'rust-lang/rust.vim'
Plugin 'vim-scripts/DoxygenToolkit.vim'
Plugin 'sheerun/vim-polyglot'

call vundle#end()

if has('nvim') || has('termguicolors')
  set termguicolors
endif

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

set textwidth=80
au FileType gitcommit setlocal tw=72
set colorcolumn=+1
set formatoptions-=t
set mouse=
" Open new split panes to right and bottom, which feels more natural
set splitbelow
set splitright
set switchbuf=usetab,newtab

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
" set number
syntax enable
filetype plugin indent on
set cindent
set cinoptions=h1,l1,g1,t0,i2s,(0,w1,W2
" set t_Co=256
set background=dark
colorscheme koehler
set spellfile=~/.vimspell.add
au BufReadPost * if line("'\"") > 0 && line("'\"") <= line("$")
      \| exe "normal! g'\"" | endif
au FileType plaintex,tex set spell
au FileType plaintex,tex syntax spell toplevel

if has("gui_gtk2")
  set guifont=Monospace\ 11,Fixed\ 11
endif

" Remove spaces at end of lines
" http://stackoverflow.com/questions/356126/how-can-you-automatically-remove-trailing-whitespace-in-vim/1618401#1618401
fun! <SID>StripTrailingWhitespaces()
  " Only strip if the b:noStripeWhitespace variable isn't set
  if exists('b:noStripWhitespace')
    return
  endif

  let l = line(".")
  let c = col(".")
  %s/\s\+$//e
  call cursor(l, c)
endfun
" au BufWritePre <buffer> :call <SID>StripTrailingWhitespaces()
au BufWritePre * :call <SID>StripTrailingWhitespaces()
au FileType diff let b:noStripWhitespace=1

" Highlight spaces at the end of lines
match Todo /\s\+$/

" fold settings
set foldmethod=syntax
set foldnestmax=1
set nofoldenable

set linebreak

" Use the standard system clipboard by default
" set clipboard=unnamedplus

" Move through wrapped lines
noremap  <silent> <Up>   gk
inoremap <silent> <Up>   <C-o>gk
noremap  <silent> <Down> gj
inoremap <silent> <Down> <C-o>gj
noremap  <silent> <Home> g<Home>
inoremap <silent> <Home> <C-o>g<Home>
noremap  <silent> <End>  g<End>
inoremap <silent> <End>  <C-o>g<End>

" ROS launch files
autocmd BufRead,BufNewFile *.launch setfiletype xml
autocmd BufRead,BufNewFile *.machine setfiletype xml

" Enable omni completion.
autocmd FileType css setlocal omnifunc=csscomplete#CompleteCSS
autocmd FileType html,markdown setlocal omnifunc=htmlcomplete#CompleteTags
autocmd FileType javascript setlocal omnifunc=javascriptcomplete#CompleteJS
autocmd FileType python setlocal omnifunc=pythoncomplete#Complete
autocmd FileType xml setlocal omnifunc=xmlcomplete#CompleteTags

let g:licenses_authors_name = 'Kartik Mohta <kartikmohta@gmail.com>'

" Set ft=tex as default for .tex files
let g:tex_flavor = "latex"

set tags=./tags;
autocmd FileType c,cpp,cuda map <C-K> :py3f /usr/share/clang/clang-format.py<CR>
autocmd FileType c,cpp,cuda imap <C-K> <c-o>:py3f /usr/share/clang/clang-format.py<CR>


" Make Vim recognize XTerm escape sequences for Arrow keys combined with
" modifiers such as Shift, Control, and Alt.  See http://superuser.com/a/402084.
if &term =~ '^screen'
  execute "set <xUp>=\e[1;*A"
  execute "set <xDown>=\e[1;*B"
  execute "set <xRight>=\e[1;*C"
  execute "set <xLeft>=\e[1;*D"
endif

" Autoload Doxygen highlighting
let g:load_doxygen_syntax = 1
