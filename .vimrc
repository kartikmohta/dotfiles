set nocompatible
filetype off

" Specify a directory for plugins
" Avoid using standard Vim directory names like 'plugin'
call plug#begin('~/.vim/plugged')

Plug 'antoyo/vim-licenses'
Plug 'vim-scripts/DoxygenToolkit.vim'
Plug 'sheerun/vim-polyglot'

Plug 'Valloric/YouCompleteMe', {'do': 'LDFLAGS= ./install.py --clangd-completer --ninja'}
Plug 'Chiel92/vim-autoformat'
Plug 'chaoren/vim-wordmotion'

" Initialize plugin system
call plug#end()


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
set inccommand=nosplit
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
  if exists('b:noStripWhitespace') || &binary
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

"set tags=./tags;
"autocmd FileType c,cpp,cuda map <C-K> :py3f /usr/share/clang/clang-format.py<CR>
"autocmd FileType c,cpp,cuda imap <C-K> <c-o>:py3f /usr/share/clang/clang-format.py<CR>
noremap <C-K> :Autoformat<CR>

" YouCompleteMe
let g:ycm_confirm_extra_conf = 0
let g:ycm_autoclose_preview_window_after_completion = 1
let g:ycm_collect_identifiers_from_tags_files = 0
map <C-P> :vertical YcmCompleter GoToImprecise<CR>
let g:ycm_global_ycm_extra_conf="/home/kartikmohta/.ycm_extra_conf.py"
let g:ycm_disable_for_files_larger_than_kb = 200
let g:ycm_max_diagnostics_to_display = 100
let g:ycm_goto_buffer_command = 'split-or-existing-window'
" Let clangd fully control code completion
let g:ycm_clangd_uses_ycmd_caching = 0
" Use installed clangd, not YCM-bundled clangd which doesn't get updates.
let g:ycm_clangd_binary_path = exepath("clangd")
let g:ycm_clangd_args = [ '--clang-tidy-checks=-*,
      \bugprone-*,
      \modernize-*,
      \performance-*,
      \-modernize-use-trailing-return-type,
      \-modernize-avoid-c-arrays',
      \'--header-insertion=never' ]

" Autoload Doxygen highlighting
let g:load_doxygen_syntax = 1

" Program to use for evaluating Python code. Setting this makes startup faster.
let g:python_host_prog="python2"
let g:python3_host_prog="python3"

" For security
set nomodeline
