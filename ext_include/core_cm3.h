





<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="utf-8">



  <link crossorigin="anonymous" href="https://assets-cdn.github.com/assets/frameworks-5b61aadc846f0818981ceec31b49c475fb084c163fdec5efbc2c21ef539092a9.css" integrity="sha256-W2Gq3IRvCBiYHO7DG0nEdfsITBY/3sXvvCwh71OQkqk=" media="all" rel="stylesheet" />
  <link crossorigin="anonymous" href="https://assets-cdn.github.com/assets/github-448c90f2e29bc363f13d0fb5cf7127f801ae7aa84a9b4f181cd43b943786ee6f.css" integrity="sha256-RIyQ8uKbw2PxPQ+1z3En+AGueqhKm08YHNQ7lDeG7m8=" media="all" rel="stylesheet" />
  
  
  <link crossorigin="anonymous" href="https://assets-cdn.github.com/assets/site-b29e324b8fafaead965049ef224818ef0dccc7384b5cfcad56a56a89c33a9438.css" integrity="sha256-sp4yS4+vrq2WUEnvIkgY7w3MxzhLXPytVqVqicM6lDg=" media="all" rel="stylesheet" />
  

  <meta name="viewport" content="width=device-width">
  
  <title>core-common-lib/core_cm3.h at master · spark/core-common-lib · GitHub</title>
  <link rel="search" type="application/opensearchdescription+xml" href="/opensearch.xml" title="GitHub">
  <link rel="fluid-icon" href="https://github.com/fluidicon.png" title="GitHub">
  <meta property="fb:app_id" content="1401488693436528">

    
    <meta content="https://avatars0.githubusercontent.com/u/2348966?v=3&amp;s=400" property="og:image" /><meta content="GitHub" property="og:site_name" /><meta content="object" property="og:type" /><meta content="spark/core-common-lib" property="og:title" /><meta content="https://github.com/spark/core-common-lib" property="og:url" /><meta content="core-common-lib - Common library for projects that use the Spark Core with the CC3000" property="og:description" />

  <link rel="assets" href="https://assets-cdn.github.com/">
  
  <meta name="pjax-timeout" content="1000">
  
  <meta name="request-id" content="CF80:0222:1FC4CE:320A71:58D2A5FA" data-pjax-transient>
  

  <meta name="selected-link" value="repo_source" data-pjax-transient>

  <meta name="google-site-verification" content="KT5gs8h0wvaagLKAVWq8bbeNwnZZK1r1XQysX3xurLU">
<meta name="google-site-verification" content="ZzhVyEFwb7w3e0-uOTltm8Jsck2F5StVihD0exw2fsA">
    <meta name="google-analytics" content="UA-3769691-2">

<meta content="collector.githubapp.com" name="octolytics-host" /><meta content="github" name="octolytics-app-id" /><meta content="https://collector.githubapp.com/github-external/browser_event" name="octolytics-event-url" /><meta content="CF80:0222:1FC4CE:320A71:58D2A5FA" name="octolytics-dimension-request_id" />
<meta content="/&lt;user-name&gt;/&lt;repo-name&gt;/blob/show" data-pjax-transient="true" name="analytics-location" />




  <meta class="js-ga-set" name="dimension1" content="Logged Out">


  

      <meta name="hostname" content="github.com">
  <meta name="user-login" content="">

      <meta name="expected-hostname" content="github.com">
    <meta name="js-proxy-site-detection-payload" content="ZjYyMDZjODc0NTQ5MDZlMDU2NGFkZjQ1MjgyOGYyNjMyNWE2NDhhZThiOTE2NjU3NzhmYmU4YzA5ZTljNjliNHx7InJlbW90ZV9hZGRyZXNzIjoiODcuOTMuNS45MiIsInJlcXVlc3RfaWQiOiJDRjgwOjAyMjI6MUZDNENFOjMyMEE3MTo1OEQyQTVGQSIsInRpbWVzdGFtcCI6MTQ5MDIwMDA2NSwiaG9zdCI6ImdpdGh1Yi5jb20ifQ==">


  <meta name="html-safe-nonce" content="1a37497f5dc720e4b06dfcd141067d85ebd579db">

  <meta http-equiv="x-pjax-version" content="c47fc29ae241bb4894f4a67e34f0e2d2">
  

    
  <meta name="description" content="core-common-lib - Common library for projects that use the Spark Core with the CC3000">
  <meta name="go-import" content="github.com/spark/core-common-lib git https://github.com/spark/core-common-lib.git">

  <meta content="2348966" name="octolytics-dimension-user_id" /><meta content="spark" name="octolytics-dimension-user_login" /><meta content="12304529" name="octolytics-dimension-repository_id" /><meta content="spark/core-common-lib" name="octolytics-dimension-repository_nwo" /><meta content="true" name="octolytics-dimension-repository_public" /><meta content="false" name="octolytics-dimension-repository_is_fork" /><meta content="12304529" name="octolytics-dimension-repository_network_root_id" /><meta content="spark/core-common-lib" name="octolytics-dimension-repository_network_root_nwo" />
        <link href="https://github.com/spark/core-common-lib/commits/master.atom" rel="alternate" title="Recent Commits to core-common-lib:master" type="application/atom+xml">


    <link rel="canonical" href="https://github.com/spark/core-common-lib/blob/master/CMSIS/Include/core_cm3.h" data-pjax-transient>


  <meta name="browser-stats-url" content="https://api.github.com/_private/browser/stats">

  <meta name="browser-errors-url" content="https://api.github.com/_private/browser/errors">

  <link rel="mask-icon" href="https://assets-cdn.github.com/pinned-octocat.svg" color="#000000">
  <link rel="icon" type="image/x-icon" href="https://assets-cdn.github.com/favicon.ico">

<meta name="theme-color" content="#1e2327">



  </head>

  <body class="logged-out env-production page-blob">
    

  <div class="position-relative js-header-wrapper ">
    <a href="#start-of-content" tabindex="1" class="accessibility-aid js-skip-to-content">Skip to content</a>
    <div id="js-pjax-loader-bar" class="pjax-loader-bar"><div class="progress"></div></div>

    
    
    



          <header class="site-header js-details-container Details" role="banner">
  <div class="container-responsive">
    <a class="header-logo-invertocat" href="https://github.com/" aria-label="Homepage" data-ga-click="(Logged out) Header, go to homepage, icon:logo-wordmark">
      <svg aria-hidden="true" class="octicon octicon-mark-github" height="32" version="1.1" viewBox="0 0 16 16" width="32"><path fill-rule="evenodd" d="M8 0C3.58 0 0 3.58 0 8c0 3.54 2.29 6.53 5.47 7.59.4.07.55-.17.55-.38 0-.19-.01-.82-.01-1.49-2.01.37-2.53-.49-2.69-.94-.09-.23-.48-.94-.82-1.13-.28-.15-.68-.52-.01-.53.63-.01 1.08.58 1.23.82.72 1.21 1.87.87 2.33.66.07-.52.28-.87.51-1.07-1.78-.2-3.64-.89-3.64-3.95 0-.87.31-1.59.82-2.15-.08-.2-.36-1.02.08-2.12 0 0 .67-.21 2.2.82.64-.18 1.32-.27 2-.27.68 0 1.36.09 2 .27 1.53-1.04 2.2-.82 2.2-.82.44 1.1.16 1.92.08 2.12.51.56.82 1.27.82 2.15 0 3.07-1.87 3.75-3.65 3.95.29.25.54.73.54 1.48 0 1.07-.01 1.93-.01 2.2 0 .21.15.46.55.38A8.013 8.013 0 0 0 16 8c0-4.42-3.58-8-8-8z"/></svg>
    </a>

    <button class="btn-link float-right site-header-toggle js-details-target" type="button" aria-label="Toggle navigation">
      <svg aria-hidden="true" class="octicon octicon-three-bars" height="24" version="1.1" viewBox="0 0 12 16" width="18"><path fill-rule="evenodd" d="M11.41 9H.59C0 9 0 8.59 0 8c0-.59 0-1 .59-1H11.4c.59 0 .59.41.59 1 0 .59 0 1-.59 1h.01zm0-4H.59C0 5 0 4.59 0 4c0-.59 0-1 .59-1H11.4c.59 0 .59.41.59 1 0 .59 0 1-.59 1h.01zM.59 11H11.4c.59 0 .59.41.59 1 0 .59 0 1-.59 1H.59C0 13 0 12.59 0 12c0-.59 0-1 .59-1z"/></svg>
    </button>

    <div class="site-header-menu">
      <nav class="site-header-nav">
        <a href="/features" class="js-selected-navigation-item nav-item" data-ga-click="Header, click, Nav menu - item:features" data-selected-links="/features /features">
          Features
</a>        <a href="/explore" class="js-selected-navigation-item nav-item" data-ga-click="Header, click, Nav menu - item:explore" data-selected-links="/explore /trending /trending/developers /integrations /integrations/feature/code /integrations/feature/collaborate /integrations/feature/ship /showcases /explore">
          Explore
</a>        <a href="/pricing" class="js-selected-navigation-item nav-item" data-ga-click="Header, click, Nav menu - item:pricing" data-selected-links="/pricing /pricing">
          Pricing
</a>      </nav>

      <div class="site-header-actions">
          <div class="header-search scoped-search site-scoped-search js-site-search" role="search">
  <!-- '"` --><!-- </textarea></xmp> --></option></form><form accept-charset="UTF-8" action="/spark/core-common-lib/search" class="js-site-search-form" data-scoped-search-url="/spark/core-common-lib/search" data-unscoped-search-url="/search" method="get"><div style="margin:0;padding:0;display:inline"><input name="utf8" type="hidden" value="&#x2713;" /></div>
    <label class="form-control header-search-wrapper js-chromeless-input-container">
        <a href="/spark/core-common-lib/blob/master/CMSIS/Include/core_cm3.h" class="header-search-scope no-underline">This repository</a>
      <input type="text"
        class="form-control header-search-input js-site-search-focus js-site-search-field is-clearable"
        data-hotkey="s"
        name="q"
        value=""
        placeholder="Search"
        aria-label="Search this repository"
        data-unscoped-placeholder="Search GitHub"
        data-scoped-placeholder="Search"
        autocapitalize="off">
        <input type="hidden" class="js-site-search-type-field" name="type" >
    </label>
</form></div>


          <a class="text-bold site-header-link" href="/login?return_to=%2Fspark%2Fcore-common-lib%2Fblob%2Fmaster%2FCMSIS%2FInclude%2Fcore_cm3.h" data-ga-click="(Logged out) Header, clicked Sign in, text:sign-in">Sign in</a>
            <span class="text-gray">or</span>
            <a class="text-bold site-header-link" href="/join?source=header-repo" data-ga-click="(Logged out) Header, clicked Sign up, text:sign-up">Sign up</a>
      </div>
    </div>
  </div>
</header>


  </div>

  <div id="start-of-content" class="accessibility-aid"></div>

    <div id="js-flash-container">
</div>



  <div role="main">
      <div itemscope itemtype="http://schema.org/SoftwareSourceCode">
    <div id="js-repo-pjax-container" data-pjax-container>
        



  <div class="pagehead repohead instapaper_ignore readability-menu experiment-repo-nav">
    <div class="container repohead-details-container">


      <ul class="pagehead-actions">
  <li>
      <a href="/login?return_to=%2Fspark%2Fcore-common-lib"
    class="btn btn-sm btn-with-count tooltipped tooltipped-n"
    aria-label="You must be signed in to watch a repository" rel="nofollow">
    <svg aria-hidden="true" class="octicon octicon-eye" height="16" version="1.1" viewBox="0 0 16 16" width="16"><path fill-rule="evenodd" d="M8.06 2C3 2 0 8 0 8s3 6 8.06 6C13 14 16 8 16 8s-3-6-7.94-6zM8 12c-2.2 0-4-1.78-4-4 0-2.2 1.8-4 4-4 2.22 0 4 1.8 4 4 0 2.22-1.78 4-4 4zm2-4c0 1.11-.89 2-2 2-1.11 0-2-.89-2-2 0-1.11.89-2 2-2 1.11 0 2 .89 2 2z"/></svg>
    Watch
  </a>
  <a class="social-count" href="/spark/core-common-lib/watchers"
     aria-label="55 users are watching this repository">
    55
  </a>

  </li>

  <li>
      <a href="/login?return_to=%2Fspark%2Fcore-common-lib"
    class="btn btn-sm btn-with-count tooltipped tooltipped-n"
    aria-label="You must be signed in to star a repository" rel="nofollow">
    <svg aria-hidden="true" class="octicon octicon-star" height="16" version="1.1" viewBox="0 0 14 16" width="14"><path fill-rule="evenodd" d="M14 6l-4.9-.64L7 1 4.9 5.36 0 6l3.6 3.26L2.67 14 7 11.67 11.33 14l-.93-4.74z"/></svg>
    Star
  </a>

    <a class="social-count js-social-count" href="/spark/core-common-lib/stargazers"
      aria-label="56 users starred this repository">
      56
    </a>

  </li>

  <li>
      <a href="/login?return_to=%2Fspark%2Fcore-common-lib"
        class="btn btn-sm btn-with-count tooltipped tooltipped-n"
        aria-label="You must be signed in to fork a repository" rel="nofollow">
        <svg aria-hidden="true" class="octicon octicon-repo-forked" height="16" version="1.1" viewBox="0 0 10 16" width="10"><path fill-rule="evenodd" d="M8 1a1.993 1.993 0 0 0-1 3.72V6L5 8 3 6V4.72A1.993 1.993 0 0 0 2 1a1.993 1.993 0 0 0-1 3.72V6.5l3 3v1.78A1.993 1.993 0 0 0 5 15a1.993 1.993 0 0 0 1-3.72V9.5l3-3V4.72A1.993 1.993 0 0 0 8 1zM2 4.2C1.34 4.2.8 3.65.8 3c0-.65.55-1.2 1.2-1.2.65 0 1.2.55 1.2 1.2 0 .65-.55 1.2-1.2 1.2zm3 10c-.66 0-1.2-.55-1.2-1.2 0-.65.55-1.2 1.2-1.2.65 0 1.2.55 1.2 1.2 0 .65-.55 1.2-1.2 1.2zm3-10c-.66 0-1.2-.55-1.2-1.2 0-.65.55-1.2 1.2-1.2.65 0 1.2.55 1.2 1.2 0 .65-.55 1.2-1.2 1.2z"/></svg>
        Fork
      </a>

    <a href="/spark/core-common-lib/network" class="social-count"
       aria-label="56 users forked this repository">
      56
    </a>
  </li>
</ul>

      <h1 class="public ">
  <svg aria-hidden="true" class="octicon octicon-repo" height="16" version="1.1" viewBox="0 0 12 16" width="12"><path fill-rule="evenodd" d="M4 9H3V8h1v1zm0-3H3v1h1V6zm0-2H3v1h1V4zm0-2H3v1h1V2zm8-1v12c0 .55-.45 1-1 1H6v2l-1.5-1.5L3 16v-2H1c-.55 0-1-.45-1-1V1c0-.55.45-1 1-1h10c.55 0 1 .45 1 1zm-1 10H1v2h2v-1h3v1h5v-2zm0-10H2v9h9V1z"/></svg>
  <span class="author" itemprop="author"><a href="/spark" class="url fn" rel="author">spark</a></span><!--
--><span class="path-divider">/</span><!--
--><strong itemprop="name"><a href="/spark/core-common-lib" data-pjax="#js-repo-pjax-container">core-common-lib</a></strong>

</h1>

    </div>
    <div class="container">
      
<nav class="reponav js-repo-nav js-sidenav-container-pjax"
     itemscope
     itemtype="http://schema.org/BreadcrumbList"
     role="navigation"
     data-pjax="#js-repo-pjax-container">

  <span itemscope itemtype="http://schema.org/ListItem" itemprop="itemListElement">
    <a href="/spark/core-common-lib" class="js-selected-navigation-item selected reponav-item" data-hotkey="g c" data-selected-links="repo_source repo_downloads repo_commits repo_releases repo_tags repo_branches /spark/core-common-lib" itemprop="url">
      <svg aria-hidden="true" class="octicon octicon-code" height="16" version="1.1" viewBox="0 0 14 16" width="14"><path fill-rule="evenodd" d="M9.5 3L8 4.5 11.5 8 8 11.5 9.5 13 14 8 9.5 3zm-5 0L0 8l4.5 5L6 11.5 2.5 8 6 4.5 4.5 3z"/></svg>
      <span itemprop="name">Code</span>
      <meta itemprop="position" content="1">
</a>  </span>

    <span itemscope itemtype="http://schema.org/ListItem" itemprop="itemListElement">
      <a href="/spark/core-common-lib/issues" class="js-selected-navigation-item reponav-item" data-hotkey="g i" data-selected-links="repo_issues repo_labels repo_milestones /spark/core-common-lib/issues" itemprop="url">
        <svg aria-hidden="true" class="octicon octicon-issue-opened" height="16" version="1.1" viewBox="0 0 14 16" width="14"><path fill-rule="evenodd" d="M7 2.3c3.14 0 5.7 2.56 5.7 5.7s-2.56 5.7-5.7 5.7A5.71 5.71 0 0 1 1.3 8c0-3.14 2.56-5.7 5.7-5.7zM7 1C3.14 1 0 4.14 0 8s3.14 7 7 7 7-3.14 7-7-3.14-7-7-7zm1 3H6v5h2V4zm0 6H6v2h2v-2z"/></svg>
        <span itemprop="name">Issues</span>
        <span class="counter">2</span>
        <meta itemprop="position" content="2">
</a>    </span>

  <span itemscope itemtype="http://schema.org/ListItem" itemprop="itemListElement">
    <a href="/spark/core-common-lib/pulls" class="js-selected-navigation-item reponav-item" data-hotkey="g p" data-selected-links="repo_pulls /spark/core-common-lib/pulls" itemprop="url">
      <svg aria-hidden="true" class="octicon octicon-git-pull-request" height="16" version="1.1" viewBox="0 0 12 16" width="12"><path fill-rule="evenodd" d="M11 11.28V5c-.03-.78-.34-1.47-.94-2.06C9.46 2.35 8.78 2.03 8 2H7V0L4 3l3 3V4h1c.27.02.48.11.69.31.21.2.3.42.31.69v6.28A1.993 1.993 0 0 0 10 15a1.993 1.993 0 0 0 1-3.72zm-1 2.92c-.66 0-1.2-.55-1.2-1.2 0-.65.55-1.2 1.2-1.2.65 0 1.2.55 1.2 1.2 0 .65-.55 1.2-1.2 1.2zM4 3c0-1.11-.89-2-2-2a1.993 1.993 0 0 0-1 3.72v6.56A1.993 1.993 0 0 0 2 15a1.993 1.993 0 0 0 1-3.72V4.72c.59-.34 1-.98 1-1.72zm-.8 10c0 .66-.55 1.2-1.2 1.2-.65 0-1.2-.55-1.2-1.2 0-.65.55-1.2 1.2-1.2.65 0 1.2.55 1.2 1.2zM2 4.2C1.34 4.2.8 3.65.8 3c0-.65.55-1.2 1.2-1.2.65 0 1.2.55 1.2 1.2 0 .65-.55 1.2-1.2 1.2z"/></svg>
      <span itemprop="name">Pull requests</span>
      <span class="counter">6</span>
      <meta itemprop="position" content="3">
</a>  </span>

  <a href="/spark/core-common-lib/projects" class="js-selected-navigation-item reponav-item" data-selected-links="repo_projects new_repo_project repo_project /spark/core-common-lib/projects">
    <svg aria-hidden="true" class="octicon octicon-project" height="16" version="1.1" viewBox="0 0 15 16" width="15"><path fill-rule="evenodd" d="M10 12h3V2h-3v10zm-4-2h3V2H6v8zm-4 4h3V2H2v12zm-1 1h13V1H1v14zM14 0H1a1 1 0 0 0-1 1v14a1 1 0 0 0 1 1h13a1 1 0 0 0 1-1V1a1 1 0 0 0-1-1z"/></svg>
    Projects
    <span class="counter">0</span>
</a>


  <a href="/spark/core-common-lib/pulse" class="js-selected-navigation-item reponav-item" data-selected-links="pulse /spark/core-common-lib/pulse">
    <svg aria-hidden="true" class="octicon octicon-pulse" height="16" version="1.1" viewBox="0 0 14 16" width="14"><path fill-rule="evenodd" d="M11.5 8L8.8 5.4 6.6 8.5 5.5 1.6 2.38 8H0v2h3.6l.9-1.8.9 5.4L9 8.5l1.6 1.5H14V8z"/></svg>
    Pulse
</a>
  <a href="/spark/core-common-lib/graphs" class="js-selected-navigation-item reponav-item" data-selected-links="repo_graphs repo_contributors /spark/core-common-lib/graphs">
    <svg aria-hidden="true" class="octicon octicon-graph" height="16" version="1.1" viewBox="0 0 16 16" width="16"><path fill-rule="evenodd" d="M16 14v1H0V0h1v14h15zM5 13H3V8h2v5zm4 0H7V3h2v10zm4 0h-2V6h2v7z"/></svg>
    Graphs
</a>

</nav>

    </div>
  </div>

<div class="container new-discussion-timeline experiment-repo-nav">
  <div class="repository-content">

    

<a href="/spark/core-common-lib/blob/3cdaed3ca40949a466027942c4b253adebc2ccf0/CMSIS/Include/core_cm3.h" class="d-none js-permalink-shortcut" data-hotkey="y">Permalink</a>

<!-- blob contrib key: blob_contributors:v21:be2bc436a026ad0355db2872640dd320 -->

<div class="file-navigation js-zeroclipboard-container">
  
<div class="select-menu branch-select-menu js-menu-container js-select-menu float-left">
  <button class=" btn btn-sm select-menu-button js-menu-target css-truncate" data-hotkey="w"
    
    type="button" aria-label="Switch branches or tags" tabindex="0" aria-haspopup="true">
      <i>Branch:</i>
      <span class="js-select-button css-truncate-target">master</span>
  </button>

  <div class="select-menu-modal-holder js-menu-content js-navigation-container" data-pjax aria-hidden="true">

    <div class="select-menu-modal">
      <div class="select-menu-header">
        <svg aria-label="Close" class="octicon octicon-x js-menu-close" height="16" role="img" version="1.1" viewBox="0 0 12 16" width="12"><path fill-rule="evenodd" d="M7.48 8l3.75 3.75-1.48 1.48L6 9.48l-3.75 3.75-1.48-1.48L4.52 8 .77 4.25l1.48-1.48L6 6.52l3.75-3.75 1.48 1.48z"/></svg>
        <span class="select-menu-title">Switch branches/tags</span>
      </div>

      <div class="select-menu-filters">
        <div class="select-menu-text-filter">
          <input type="text" aria-label="Filter branches/tags" id="context-commitish-filter-field" class="form-control js-filterable-field js-navigation-enable" placeholder="Filter branches/tags">
        </div>
        <div class="select-menu-tabs">
          <ul>
            <li class="select-menu-tab">
              <a href="#" data-tab-filter="branches" data-filter-placeholder="Filter branches/tags" class="js-select-menu-tab" role="tab">Branches</a>
            </li>
            <li class="select-menu-tab">
              <a href="#" data-tab-filter="tags" data-filter-placeholder="Find a tag…" class="js-select-menu-tab" role="tab">Tags</a>
            </li>
          </ul>
        </div>
      </div>

      <div class="select-menu-list select-menu-tab-bucket js-select-menu-tab-bucket" data-tab-filter="branches" role="menu">

        <div data-filterable-for="context-commitish-filter-field" data-filterable-type="substring">


            <a class="select-menu-item js-navigation-item js-navigation-open "
               href="/spark/core-common-lib/blob/arp-event-fix/CMSIS/Include/core_cm3.h"
               data-name="arp-event-fix"
               data-skip-pjax="true"
               rel="nofollow">
              <svg aria-hidden="true" class="octicon octicon-check select-menu-item-icon" height="16" version="1.1" viewBox="0 0 12 16" width="12"><path fill-rule="evenodd" d="M12 5l-8 8-4-4 1.5-1.5L4 10l6.5-6.5z"/></svg>
              <span class="select-menu-item-text css-truncate-target js-select-menu-filter-text">
                arp-event-fix
              </span>
            </a>
            <a class="select-menu-item js-navigation-item js-navigation-open "
               href="/spark/core-common-lib/blob/compile-server2/CMSIS/Include/core_cm3.h"
               data-name="compile-server2"
               data-skip-pjax="true"
               rel="nofollow">
              <svg aria-hidden="true" class="octicon octicon-check select-menu-item-icon" height="16" version="1.1" viewBox="0 0 12 16" width="12"><path fill-rule="evenodd" d="M12 5l-8 8-4-4 1.5-1.5L4 10l6.5-6.5z"/></svg>
              <span class="select-menu-item-text css-truncate-target js-select-menu-filter-text">
                compile-server2
              </span>
            </a>
            <a class="select-menu-item js-navigation-item js-navigation-open "
               href="/spark/core-common-lib/blob/compile-staging/CMSIS/Include/core_cm3.h"
               data-name="compile-staging"
               data-skip-pjax="true"
               rel="nofollow">
              <svg aria-hidden="true" class="octicon octicon-check select-menu-item-icon" height="16" version="1.1" viewBox="0 0 12 16" width="12"><path fill-rule="evenodd" d="M12 5l-8 8-4-4 1.5-1.5L4 10l6.5-6.5z"/></svg>
              <span class="select-menu-item-text css-truncate-target js-select-menu-filter-text">
                compile-staging
              </span>
            </a>
            <a class="select-menu-item js-navigation-item js-navigation-open "
               href="/spark/core-common-lib/blob/context-switching/CMSIS/Include/core_cm3.h"
               data-name="context-switching"
               data-skip-pjax="true"
               rel="nofollow">
              <svg aria-hidden="true" class="octicon octicon-check select-menu-item-icon" height="16" version="1.1" viewBox="0 0 12 16" width="12"><path fill-rule="evenodd" d="M12 5l-8 8-4-4 1.5-1.5L4 10l6.5-6.5z"/></svg>
              <span class="select-menu-item-text css-truncate-target js-select-menu-filter-text">
                context-switching
              </span>
            </a>
            <a class="select-menu-item js-navigation-item js-navigation-open "
               href="/spark/core-common-lib/blob/feature/flasher_additions/CMSIS/Include/core_cm3.h"
               data-name="feature/flasher_additions"
               data-skip-pjax="true"
               rel="nofollow">
              <svg aria-hidden="true" class="octicon octicon-check select-menu-item-icon" height="16" version="1.1" viewBox="0 0 12 16" width="12"><path fill-rule="evenodd" d="M12 5l-8 8-4-4 1.5-1.5L4 10l6.5-6.5z"/></svg>
              <span class="select-menu-item-text css-truncate-target js-select-menu-filter-text">
                feature/flasher_additions
              </span>
            </a>
            <a class="select-menu-item js-navigation-item js-navigation-open "
               href="/spark/core-common-lib/blob/feature/hal/CMSIS/Include/core_cm3.h"
               data-name="feature/hal"
               data-skip-pjax="true"
               rel="nofollow">
              <svg aria-hidden="true" class="octicon octicon-check select-menu-item-icon" height="16" version="1.1" viewBox="0 0 12 16" width="12"><path fill-rule="evenodd" d="M12 5l-8 8-4-4 1.5-1.5L4 10l6.5-6.5z"/></svg>
              <span class="select-menu-item-text css-truncate-target js-select-menu-filter-text">
                feature/hal
              </span>
            </a>
            <a class="select-menu-item js-navigation-item js-navigation-open "
               href="/spark/core-common-lib/blob/feature/led-change-notify/CMSIS/Include/core_cm3.h"
               data-name="feature/led-change-notify"
               data-skip-pjax="true"
               rel="nofollow">
              <svg aria-hidden="true" class="octicon octicon-check select-menu-item-icon" height="16" version="1.1" viewBox="0 0 12 16" width="12"><path fill-rule="evenodd" d="M12 5l-8 8-4-4 1.5-1.5L4 10l6.5-6.5z"/></svg>
              <span class="select-menu-item-text css-truncate-target js-select-menu-filter-text">
                feature/led-change-notify
              </span>
            </a>
            <a class="select-menu-item js-navigation-item js-navigation-open "
               href="/spark/core-common-lib/blob/feature/sdcard-fatfs-support/CMSIS/Include/core_cm3.h"
               data-name="feature/sdcard-fatfs-support"
               data-skip-pjax="true"
               rel="nofollow">
              <svg aria-hidden="true" class="octicon octicon-check select-menu-item-icon" height="16" version="1.1" viewBox="0 0 12 16" width="12"><path fill-rule="evenodd" d="M12 5l-8 8-4-4 1.5-1.5L4 10l6.5-6.5z"/></svg>
              <span class="select-menu-item-text css-truncate-target js-select-menu-filter-text">
                feature/sdcard-fatfs-support
              </span>
            </a>
            <a class="select-menu-item js-navigation-item js-navigation-open "
               href="/spark/core-common-lib/blob/feature/spark-product-id/CMSIS/Include/core_cm3.h"
               data-name="feature/spark-product-id"
               data-skip-pjax="true"
               rel="nofollow">
              <svg aria-hidden="true" class="octicon octicon-check select-menu-item-icon" height="16" version="1.1" viewBox="0 0 12 16" width="12"><path fill-rule="evenodd" d="M12 5l-8 8-4-4 1.5-1.5L4 10l6.5-6.5z"/></svg>
              <span class="select-menu-item-text css-truncate-target js-select-menu-filter-text">
                feature/spark-product-id
              </span>
            </a>
            <a class="select-menu-item js-navigation-item js-navigation-open "
               href="/spark/core-common-lib/blob/feature/stm32-dsp-lib/CMSIS/Include/core_cm3.h"
               data-name="feature/stm32-dsp-lib"
               data-skip-pjax="true"
               rel="nofollow">
              <svg aria-hidden="true" class="octicon octicon-check select-menu-item-icon" height="16" version="1.1" viewBox="0 0 12 16" width="12"><path fill-rule="evenodd" d="M12 5l-8 8-4-4 1.5-1.5L4 10l6.5-6.5z"/></svg>
              <span class="select-menu-item-text css-truncate-target js-select-menu-filter-text">
                feature/stm32-dsp-lib
              </span>
            </a>
            <a class="select-menu-item js-navigation-item js-navigation-open selected"
               href="/spark/core-common-lib/blob/master/CMSIS/Include/core_cm3.h"
               data-name="master"
               data-skip-pjax="true"
               rel="nofollow">
              <svg aria-hidden="true" class="octicon octicon-check select-menu-item-icon" height="16" version="1.1" viewBox="0 0 12 16" width="12"><path fill-rule="evenodd" d="M12 5l-8 8-4-4 1.5-1.5L4 10l6.5-6.5z"/></svg>
              <span class="select-menu-item-text css-truncate-target js-select-menu-filter-text">
                master
              </span>
            </a>
            <a class="select-menu-item js-navigation-item js-navigation-open "
               href="/spark/core-common-lib/blob/refactor/usb-hal/CMSIS/Include/core_cm3.h"
               data-name="refactor/usb-hal"
               data-skip-pjax="true"
               rel="nofollow">
              <svg aria-hidden="true" class="octicon octicon-check select-menu-item-icon" height="16" version="1.1" viewBox="0 0 12 16" width="12"><path fill-rule="evenodd" d="M12 5l-8 8-4-4 1.5-1.5L4 10l6.5-6.5z"/></svg>
              <span class="select-menu-item-text css-truncate-target js-select-menu-filter-text">
                refactor/usb-hal
              </span>
            </a>
        </div>

          <div class="select-menu-no-results">Nothing to show</div>
      </div>

      <div class="select-menu-list select-menu-tab-bucket js-select-menu-tab-bucket" data-tab-filter="tags">
        <div data-filterable-for="context-commitish-filter-field" data-filterable-type="substring">


            <a class="select-menu-item js-navigation-item js-navigation-open "
              href="/spark/core-common-lib/tree/spark_11/CMSIS/Include/core_cm3.h"
              data-name="spark_11"
              data-skip-pjax="true"
              rel="nofollow">
              <svg aria-hidden="true" class="octicon octicon-check select-menu-item-icon" height="16" version="1.1" viewBox="0 0 12 16" width="12"><path fill-rule="evenodd" d="M12 5l-8 8-4-4 1.5-1.5L4 10l6.5-6.5z"/></svg>
              <span class="select-menu-item-text css-truncate-target" title="spark_11">
                spark_11
              </span>
            </a>
            <a class="select-menu-item js-navigation-item js-navigation-open "
              href="/spark/core-common-lib/tree/spark_10/CMSIS/Include/core_cm3.h"
              data-name="spark_10"
              data-skip-pjax="true"
              rel="nofollow">
              <svg aria-hidden="true" class="octicon octicon-check select-menu-item-icon" height="16" version="1.1" viewBox="0 0 12 16" width="12"><path fill-rule="evenodd" d="M12 5l-8 8-4-4 1.5-1.5L4 10l6.5-6.5z"/></svg>
              <span class="select-menu-item-text css-truncate-target" title="spark_10">
                spark_10
              </span>
            </a>
            <a class="select-menu-item js-navigation-item js-navigation-open "
              href="/spark/core-common-lib/tree/spark_9/CMSIS/Include/core_cm3.h"
              data-name="spark_9"
              data-skip-pjax="true"
              rel="nofollow">
              <svg aria-hidden="true" class="octicon octicon-check select-menu-item-icon" height="16" version="1.1" viewBox="0 0 12 16" width="12"><path fill-rule="evenodd" d="M12 5l-8 8-4-4 1.5-1.5L4 10l6.5-6.5z"/></svg>
              <span class="select-menu-item-text css-truncate-target" title="spark_9">
                spark_9
              </span>
            </a>
            <a class="select-menu-item js-navigation-item js-navigation-open "
              href="/spark/core-common-lib/tree/spark_8/CMSIS/Include/core_cm3.h"
              data-name="spark_8"
              data-skip-pjax="true"
              rel="nofollow">
              <svg aria-hidden="true" class="octicon octicon-check select-menu-item-icon" height="16" version="1.1" viewBox="0 0 12 16" width="12"><path fill-rule="evenodd" d="M12 5l-8 8-4-4 1.5-1.5L4 10l6.5-6.5z"/></svg>
              <span class="select-menu-item-text css-truncate-target" title="spark_8">
                spark_8
              </span>
            </a>
            <a class="select-menu-item js-navigation-item js-navigation-open "
              href="/spark/core-common-lib/tree/spark_7/CMSIS/Include/core_cm3.h"
              data-name="spark_7"
              data-skip-pjax="true"
              rel="nofollow">
              <svg aria-hidden="true" class="octicon octicon-check select-menu-item-icon" height="16" version="1.1" viewBox="0 0 12 16" width="12"><path fill-rule="evenodd" d="M12 5l-8 8-4-4 1.5-1.5L4 10l6.5-6.5z"/></svg>
              <span class="select-menu-item-text css-truncate-target" title="spark_7">
                spark_7
              </span>
            </a>
            <a class="select-menu-item js-navigation-item js-navigation-open "
              href="/spark/core-common-lib/tree/spark_6/CMSIS/Include/core_cm3.h"
              data-name="spark_6"
              data-skip-pjax="true"
              rel="nofollow">
              <svg aria-hidden="true" class="octicon octicon-check select-menu-item-icon" height="16" version="1.1" viewBox="0 0 12 16" width="12"><path fill-rule="evenodd" d="M12 5l-8 8-4-4 1.5-1.5L4 10l6.5-6.5z"/></svg>
              <span class="select-menu-item-text css-truncate-target" title="spark_6">
                spark_6
              </span>
            </a>
            <a class="select-menu-item js-navigation-item js-navigation-open "
              href="/spark/core-common-lib/tree/spark_5/CMSIS/Include/core_cm3.h"
              data-name="spark_5"
              data-skip-pjax="true"
              rel="nofollow">
              <svg aria-hidden="true" class="octicon octicon-check select-menu-item-icon" height="16" version="1.1" viewBox="0 0 12 16" width="12"><path fill-rule="evenodd" d="M12 5l-8 8-4-4 1.5-1.5L4 10l6.5-6.5z"/></svg>
              <span class="select-menu-item-text css-truncate-target" title="spark_5">
                spark_5
              </span>
            </a>
            <a class="select-menu-item js-navigation-item js-navigation-open "
              href="/spark/core-common-lib/tree/spark_4/CMSIS/Include/core_cm3.h"
              data-name="spark_4"
              data-skip-pjax="true"
              rel="nofollow">
              <svg aria-hidden="true" class="octicon octicon-check select-menu-item-icon" height="16" version="1.1" viewBox="0 0 12 16" width="12"><path fill-rule="evenodd" d="M12 5l-8 8-4-4 1.5-1.5L4 10l6.5-6.5z"/></svg>
              <span class="select-menu-item-text css-truncate-target" title="spark_4">
                spark_4
              </span>
            </a>
            <a class="select-menu-item js-navigation-item js-navigation-open "
              href="/spark/core-common-lib/tree/spark_3/CMSIS/Include/core_cm3.h"
              data-name="spark_3"
              data-skip-pjax="true"
              rel="nofollow">
              <svg aria-hidden="true" class="octicon octicon-check select-menu-item-icon" height="16" version="1.1" viewBox="0 0 12 16" width="12"><path fill-rule="evenodd" d="M12 5l-8 8-4-4 1.5-1.5L4 10l6.5-6.5z"/></svg>
              <span class="select-menu-item-text css-truncate-target" title="spark_3">
                spark_3
              </span>
            </a>
        </div>

        <div class="select-menu-no-results">Nothing to show</div>
      </div>

    </div>
  </div>
</div>

  <div class="BtnGroup float-right">
    <a href="/spark/core-common-lib/find/master"
          class="js-pjax-capture-input btn btn-sm BtnGroup-item"
          data-pjax
          data-hotkey="t">
      Find file
    </a>
    <button aria-label="Copy file path to clipboard" class="js-zeroclipboard btn btn-sm BtnGroup-item tooltipped tooltipped-s" data-copied-hint="Copied!" type="button">Copy path</button>
  </div>
  <div class="breadcrumb js-zeroclipboard-target">
    <span class="repo-root js-repo-root"><span class="js-path-segment"><a href="/spark/core-common-lib"><span>core-common-lib</span></a></span></span><span class="separator">/</span><span class="js-path-segment"><a href="/spark/core-common-lib/tree/master/CMSIS"><span>CMSIS</span></a></span><span class="separator">/</span><span class="js-path-segment"><a href="/spark/core-common-lib/tree/master/CMSIS/Include"><span>Include</span></a></span><span class="separator">/</span><strong class="final-path">core_cm3.h</strong>
  </div>
</div>



  <div class="commit-tease">
      <span class="float-right">
        <a class="commit-tease-sha" href="/spark/core-common-lib/commit/1c1ea87036bd883c3e93583657fe954687961db6" data-pjax>
          1c1ea87
        </a>
        <relative-time datetime="2013-08-23T12:50:45Z">Aug 23, 2013</relative-time>
      </span>
      <div>
        <img alt="@satishgn" class="avatar" height="20" src="https://avatars3.githubusercontent.com/u/3815573?v=3&amp;s=40" width="20" />
        <a href="/satishgn" class="user-mention" rel="contributor">satishgn</a>
          <a href="/spark/core-common-lib/commit/1c1ea87036bd883c3e93583657fe954687961db6" class="message" data-pjax="true" title="Added common source files - used across all firmware projects (WIP)">Added common source files - used across all firmware projects (WIP)</a>
      </div>

    <div class="commit-tease-contributors">
      <button type="button" class="btn-link muted-link contributors-toggle" data-facebox="#blob_contributors_box">
        <strong>1</strong>
         contributor
      </button>
      
    </div>

    <div id="blob_contributors_box" style="display:none">
      <h2 class="facebox-header" data-facebox-id="facebox-header">Users who have contributed to this file</h2>
      <ul class="facebox-user-list" data-facebox-id="facebox-description">
          <li class="facebox-user-list-item">
            <img alt="@satishgn" height="24" src="https://avatars1.githubusercontent.com/u/3815573?v=3&amp;s=48" width="24" />
            <a href="/satishgn">satishgn</a>
          </li>
      </ul>
    </div>
  </div>

<div class="file">
  <div class="file-header">
  <div class="file-actions">

    <div class="BtnGroup">
      <a href="/spark/core-common-lib/raw/master/CMSIS/Include/core_cm3.h" class="btn btn-sm BtnGroup-item" id="raw-url">Raw</a>
        <a href="/spark/core-common-lib/blame/master/CMSIS/Include/core_cm3.h" class="btn btn-sm js-update-url-with-hash BtnGroup-item" data-hotkey="b">Blame</a>
      <a href="/spark/core-common-lib/commits/master/CMSIS/Include/core_cm3.h" class="btn btn-sm BtnGroup-item" rel="nofollow">History</a>
    </div>


        <button type="button" class="btn-octicon disabled tooltipped tooltipped-nw"
          aria-label="You must be signed in to make or propose changes">
          <svg aria-hidden="true" class="octicon octicon-pencil" height="16" version="1.1" viewBox="0 0 14 16" width="14"><path fill-rule="evenodd" d="M0 12v3h3l8-8-3-3-8 8zm3 2H1v-2h1v1h1v1zm10.3-9.3L12 6 9 3l1.3-1.3a.996.996 0 0 1 1.41 0l1.59 1.59c.39.39.39 1.02 0 1.41z"/></svg>
        </button>
        <button type="button" class="btn-octicon btn-octicon-danger disabled tooltipped tooltipped-nw"
          aria-label="You must be signed in to make or propose changes">
          <svg aria-hidden="true" class="octicon octicon-trashcan" height="16" version="1.1" viewBox="0 0 12 16" width="12"><path fill-rule="evenodd" d="M11 2H9c0-.55-.45-1-1-1H5c-.55 0-1 .45-1 1H2c-.55 0-1 .45-1 1v1c0 .55.45 1 1 1v9c0 .55.45 1 1 1h7c.55 0 1-.45 1-1V5c.55 0 1-.45 1-1V3c0-.55-.45-1-1-1zm-1 12H3V5h1v8h1V5h1v8h1V5h1v8h1V5h1v9zm1-10H2V3h9v1z"/></svg>
        </button>
  </div>

  <div class="file-info">
      1613 lines (1217 sloc)
      <span class="file-info-divider"></span>
    95.1 KB
  </div>
</div>

  

  <div itemprop="text" class="blob-wrapper data type-c">
      <table class="highlight tab-size js-file-line-container" data-tab-size="8">
      <tr>
        <td id="L1" class="blob-num js-line-number" data-line-number="1"></td>
        <td id="LC1" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>************************************************************************<span class="pl-c">*/</span><span class="pl-c">/*</span>*</span></td>
      </tr>
      <tr>
        <td id="L2" class="blob-num js-line-number" data-line-number="2"></td>
        <td id="LC2" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> * @file     core_cm3.h</span></td>
      </tr>
      <tr>
        <td id="L3" class="blob-num js-line-number" data-line-number="3"></td>
        <td id="LC3" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> * @brief    CMSIS Cortex-M3 Core Peripheral Access Layer Header File</span></td>
      </tr>
      <tr>
        <td id="L4" class="blob-num js-line-number" data-line-number="4"></td>
        <td id="LC4" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> * @version  V3.01</span></td>
      </tr>
      <tr>
        <td id="L5" class="blob-num js-line-number" data-line-number="5"></td>
        <td id="LC5" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> * @date     22. March 2012</span></td>
      </tr>
      <tr>
        <td id="L6" class="blob-num js-line-number" data-line-number="6"></td>
        <td id="LC6" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> *</span></td>
      </tr>
      <tr>
        <td id="L7" class="blob-num js-line-number" data-line-number="7"></td>
        <td id="LC7" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> * @note</span></td>
      </tr>
      <tr>
        <td id="L8" class="blob-num js-line-number" data-line-number="8"></td>
        <td id="LC8" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> * Copyright (C) 2009-2012 ARM Limited. All rights reserved.</span></td>
      </tr>
      <tr>
        <td id="L9" class="blob-num js-line-number" data-line-number="9"></td>
        <td id="LC9" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> *</span></td>
      </tr>
      <tr>
        <td id="L10" class="blob-num js-line-number" data-line-number="10"></td>
        <td id="LC10" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> * @par</span></td>
      </tr>
      <tr>
        <td id="L11" class="blob-num js-line-number" data-line-number="11"></td>
        <td id="LC11" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> * ARM Limited (ARM) is supplying this software for use with Cortex-M</span></td>
      </tr>
      <tr>
        <td id="L12" class="blob-num js-line-number" data-line-number="12"></td>
        <td id="LC12" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> * processor based microcontrollers.  This file can be freely distributed</span></td>
      </tr>
      <tr>
        <td id="L13" class="blob-num js-line-number" data-line-number="13"></td>
        <td id="LC13" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> * within development tools that are supporting such ARM based processors.</span></td>
      </tr>
      <tr>
        <td id="L14" class="blob-num js-line-number" data-line-number="14"></td>
        <td id="LC14" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> *</span></td>
      </tr>
      <tr>
        <td id="L15" class="blob-num js-line-number" data-line-number="15"></td>
        <td id="LC15" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> * @par</span></td>
      </tr>
      <tr>
        <td id="L16" class="blob-num js-line-number" data-line-number="16"></td>
        <td id="LC16" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> * THIS SOFTWARE IS PROVIDED &quot;AS IS&quot;.  NO WARRANTIES, WHETHER EXPRESS, IMPLIED</span></td>
      </tr>
      <tr>
        <td id="L17" class="blob-num js-line-number" data-line-number="17"></td>
        <td id="LC17" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF</span></td>
      </tr>
      <tr>
        <td id="L18" class="blob-num js-line-number" data-line-number="18"></td>
        <td id="LC18" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.</span></td>
      </tr>
      <tr>
        <td id="L19" class="blob-num js-line-number" data-line-number="19"></td>
        <td id="LC19" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR</span></td>
      </tr>
      <tr>
        <td id="L20" class="blob-num js-line-number" data-line-number="20"></td>
        <td id="LC20" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.</span></td>
      </tr>
      <tr>
        <td id="L21" class="blob-num js-line-number" data-line-number="21"></td>
        <td id="LC21" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> *</span></td>
      </tr>
      <tr>
        <td id="L22" class="blob-num js-line-number" data-line-number="22"></td>
        <td id="LC22" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> *****************************************************************************<span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L23" class="blob-num js-line-number" data-line-number="23"></td>
        <td id="LC23" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">if</span> defined ( __ICCARM__ )</td>
      </tr>
      <tr>
        <td id="L24" class="blob-num js-line-number" data-line-number="24"></td>
        <td id="LC24" class="blob-code blob-code-inner js-file-line"> #<span class="pl-k">pragma</span> system_include  <span class="pl-c"><span class="pl-c">/*</span> treat file as system include file for MISRA check <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L25" class="blob-num js-line-number" data-line-number="25"></td>
        <td id="LC25" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">endif</span></td>
      </tr>
      <tr>
        <td id="L26" class="blob-num js-line-number" data-line-number="26"></td>
        <td id="LC26" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L27" class="blob-num js-line-number" data-line-number="27"></td>
        <td id="LC27" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">ifdef</span> __cplusplus</td>
      </tr>
      <tr>
        <td id="L28" class="blob-num js-line-number" data-line-number="28"></td>
        <td id="LC28" class="blob-code blob-code-inner js-file-line"> <span class="pl-k">extern</span> <span class="pl-s"><span class="pl-pds">&quot;</span>C<span class="pl-pds">&quot;</span></span> {</td>
      </tr>
      <tr>
        <td id="L29" class="blob-num js-line-number" data-line-number="29"></td>
        <td id="LC29" class="blob-code blob-code-inner js-file-line">#endif</td>
      </tr>
      <tr>
        <td id="L30" class="blob-num js-line-number" data-line-number="30"></td>
        <td id="LC30" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L31" class="blob-num js-line-number" data-line-number="31"></td>
        <td id="LC31" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">ifndef</span> __CORE_CM3_H_GENERIC</td>
      </tr>
      <tr>
        <td id="L32" class="blob-num js-line-number" data-line-number="32"></td>
        <td id="LC32" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">__CORE_CM3_H_GENERIC</span></td>
      </tr>
      <tr>
        <td id="L33" class="blob-num js-line-number" data-line-number="33"></td>
        <td id="LC33" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L34" class="blob-num js-line-number" data-line-number="34"></td>
        <td id="LC34" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \page CMSIS_MISRA_Exceptions  MISRA-C:2004 Compliance Exceptions</span></td>
      </tr>
      <tr>
        <td id="L35" class="blob-num js-line-number" data-line-number="35"></td>
        <td id="LC35" class="blob-code blob-code-inner js-file-line"><span class="pl-c">  CMSIS violates the following MISRA-C:2004 rules:</span></td>
      </tr>
      <tr>
        <td id="L36" class="blob-num js-line-number" data-line-number="36"></td>
        <td id="LC36" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L37" class="blob-num js-line-number" data-line-number="37"></td>
        <td id="LC37" class="blob-code blob-code-inner js-file-line"><span class="pl-c">   \li Required Rule 8.5, object/function definition in header file.&lt;br&gt;</span></td>
      </tr>
      <tr>
        <td id="L38" class="blob-num js-line-number" data-line-number="38"></td>
        <td id="LC38" class="blob-code blob-code-inner js-file-line"><span class="pl-c">     Function definitions in header files are used to allow &#39;inlining&#39;.</span></td>
      </tr>
      <tr>
        <td id="L39" class="blob-num js-line-number" data-line-number="39"></td>
        <td id="LC39" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L40" class="blob-num js-line-number" data-line-number="40"></td>
        <td id="LC40" class="blob-code blob-code-inner js-file-line"><span class="pl-c">   \li Required Rule 18.4, declaration of union type or object of union type: &#39;{...}&#39;.&lt;br&gt;</span></td>
      </tr>
      <tr>
        <td id="L41" class="blob-num js-line-number" data-line-number="41"></td>
        <td id="LC41" class="blob-code blob-code-inner js-file-line"><span class="pl-c">     Unions are used for effective representation of core registers.</span></td>
      </tr>
      <tr>
        <td id="L42" class="blob-num js-line-number" data-line-number="42"></td>
        <td id="LC42" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L43" class="blob-num js-line-number" data-line-number="43"></td>
        <td id="LC43" class="blob-code blob-code-inner js-file-line"><span class="pl-c">   \li Advisory Rule 19.7, Function-like macro defined.&lt;br&gt;</span></td>
      </tr>
      <tr>
        <td id="L44" class="blob-num js-line-number" data-line-number="44"></td>
        <td id="LC44" class="blob-code blob-code-inner js-file-line"><span class="pl-c">     Function-like macros are used to allow more efficient code.</span></td>
      </tr>
      <tr>
        <td id="L45" class="blob-num js-line-number" data-line-number="45"></td>
        <td id="LC45" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L46" class="blob-num js-line-number" data-line-number="46"></td>
        <td id="LC46" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L47" class="blob-num js-line-number" data-line-number="47"></td>
        <td id="LC47" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L48" class="blob-num js-line-number" data-line-number="48"></td>
        <td id="LC48" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>******************************************************************************</span></td>
      </tr>
      <tr>
        <td id="L49" class="blob-num js-line-number" data-line-number="49"></td>
        <td id="LC49" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> *                 CMSIS definitions</span></td>
      </tr>
      <tr>
        <td id="L50" class="blob-num js-line-number" data-line-number="50"></td>
        <td id="LC50" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> *****************************************************************************<span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L51" class="blob-num js-line-number" data-line-number="51"></td>
        <td id="LC51" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \ingroup Cortex_M3</span></td>
      </tr>
      <tr>
        <td id="L52" class="blob-num js-line-number" data-line-number="52"></td>
        <td id="LC52" class="blob-code blob-code-inner js-file-line"><span class="pl-c">  @{</span></td>
      </tr>
      <tr>
        <td id="L53" class="blob-num js-line-number" data-line-number="53"></td>
        <td id="LC53" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L54" class="blob-num js-line-number" data-line-number="54"></td>
        <td id="LC54" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L55" class="blob-num js-line-number" data-line-number="55"></td>
        <td id="LC55" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>  CMSIS CM3 definitions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L56" class="blob-num js-line-number" data-line-number="56"></td>
        <td id="LC56" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">__CM3_CMSIS_VERSION_MAIN</span>  (0x03)                                   <span class="pl-c"><span class="pl-c">/*</span>!&lt; [31:16] CMSIS HAL main version   <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L57" class="blob-num js-line-number" data-line-number="57"></td>
        <td id="LC57" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">__CM3_CMSIS_VERSION_SUB</span>   (0x01)                                   <span class="pl-c"><span class="pl-c">/*</span>!&lt; [15:0]  CMSIS HAL sub version    <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L58" class="blob-num js-line-number" data-line-number="58"></td>
        <td id="LC58" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">__CM3_CMSIS_VERSION</span>       ((__CM3_CMSIS_VERSION_MAIN &lt;&lt; <span class="pl-c1">16</span>) | \</td>
      </tr>
      <tr>
        <td id="L59" class="blob-num js-line-number" data-line-number="59"></td>
        <td id="LC59" class="blob-code blob-code-inner js-file-line">                                    __CM3_CMSIS_VERSION_SUB          )     <span class="pl-c"><span class="pl-c">/*</span>!&lt; CMSIS HAL version number         <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L60" class="blob-num js-line-number" data-line-number="60"></td>
        <td id="LC60" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L61" class="blob-num js-line-number" data-line-number="61"></td>
        <td id="LC61" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">__CORTEX_M</span>                (0x03)                                   <span class="pl-c"><span class="pl-c">/*</span>!&lt; Cortex-M Core                    <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L62" class="blob-num js-line-number" data-line-number="62"></td>
        <td id="LC62" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L63" class="blob-num js-line-number" data-line-number="63"></td>
        <td id="LC63" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L64" class="blob-num js-line-number" data-line-number="64"></td>
        <td id="LC64" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">if</span>   defined ( __CC_ARM )</td>
      </tr>
      <tr>
        <td id="L65" class="blob-num js-line-number" data-line-number="65"></td>
        <td id="LC65" class="blob-code blob-code-inner js-file-line">  #<span class="pl-k">define</span> <span class="pl-en">__ASM</span>            __asm                                      <span class="pl-c"><span class="pl-c">/*</span>!&lt; asm keyword for ARM Compiler          <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L66" class="blob-num js-line-number" data-line-number="66"></td>
        <td id="LC66" class="blob-code blob-code-inner js-file-line">  #<span class="pl-k">define</span> <span class="pl-en">__INLINE</span>         __inline                                   <span class="pl-c"><span class="pl-c">/*</span>!&lt; inline keyword for ARM Compiler       <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L67" class="blob-num js-line-number" data-line-number="67"></td>
        <td id="LC67" class="blob-code blob-code-inner js-file-line">  #<span class="pl-k">define</span> <span class="pl-en">__STATIC_INLINE</span>  <span class="pl-k">static</span> __inline</td>
      </tr>
      <tr>
        <td id="L68" class="blob-num js-line-number" data-line-number="68"></td>
        <td id="LC68" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L69" class="blob-num js-line-number" data-line-number="69"></td>
        <td id="LC69" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">elif</span> defined ( __ICCARM__ )</td>
      </tr>
      <tr>
        <td id="L70" class="blob-num js-line-number" data-line-number="70"></td>
        <td id="LC70" class="blob-code blob-code-inner js-file-line">  #<span class="pl-k">define</span> <span class="pl-en">__ASM</span>            __asm                                      <span class="pl-c"><span class="pl-c">/*</span>!&lt; asm keyword for IAR Compiler          <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L71" class="blob-num js-line-number" data-line-number="71"></td>
        <td id="LC71" class="blob-code blob-code-inner js-file-line">  #<span class="pl-k">define</span> <span class="pl-en">__INLINE</span>         <span class="pl-k">inline</span>                                     <span class="pl-c"><span class="pl-c">/*</span>!&lt; inline keyword for IAR Compiler. Only available in High optimization mode! <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L72" class="blob-num js-line-number" data-line-number="72"></td>
        <td id="LC72" class="blob-code blob-code-inner js-file-line">  #<span class="pl-k">define</span> <span class="pl-en">__STATIC_INLINE</span>  <span class="pl-k">static</span> <span class="pl-k">inline</span></td>
      </tr>
      <tr>
        <td id="L73" class="blob-num js-line-number" data-line-number="73"></td>
        <td id="LC73" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L74" class="blob-num js-line-number" data-line-number="74"></td>
        <td id="LC74" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">elif</span> defined ( __TMS470__ )</td>
      </tr>
      <tr>
        <td id="L75" class="blob-num js-line-number" data-line-number="75"></td>
        <td id="LC75" class="blob-code blob-code-inner js-file-line">  #<span class="pl-k">define</span> <span class="pl-en">__ASM</span>            __asm                                      <span class="pl-c"><span class="pl-c">/*</span>!&lt; asm keyword for TI CCS Compiler       <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L76" class="blob-num js-line-number" data-line-number="76"></td>
        <td id="LC76" class="blob-code blob-code-inner js-file-line">  #<span class="pl-k">define</span> <span class="pl-en">__STATIC_INLINE</span>  <span class="pl-k">static</span> <span class="pl-k">inline</span></td>
      </tr>
      <tr>
        <td id="L77" class="blob-num js-line-number" data-line-number="77"></td>
        <td id="LC77" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L78" class="blob-num js-line-number" data-line-number="78"></td>
        <td id="LC78" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">elif</span> defined ( __GNUC__ )</td>
      </tr>
      <tr>
        <td id="L79" class="blob-num js-line-number" data-line-number="79"></td>
        <td id="LC79" class="blob-code blob-code-inner js-file-line">  #<span class="pl-k">define</span> <span class="pl-en">__ASM</span>            __asm                                      <span class="pl-c"><span class="pl-c">/*</span>!&lt; asm keyword for GNU Compiler          <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L80" class="blob-num js-line-number" data-line-number="80"></td>
        <td id="LC80" class="blob-code blob-code-inner js-file-line">  #<span class="pl-k">define</span> <span class="pl-en">__INLINE</span>         <span class="pl-k">inline</span>                                     <span class="pl-c"><span class="pl-c">/*</span>!&lt; inline keyword for GNU Compiler       <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L81" class="blob-num js-line-number" data-line-number="81"></td>
        <td id="LC81" class="blob-code blob-code-inner js-file-line">  #<span class="pl-k">define</span> <span class="pl-en">__STATIC_INLINE</span>  <span class="pl-k">static</span> <span class="pl-k">inline</span></td>
      </tr>
      <tr>
        <td id="L82" class="blob-num js-line-number" data-line-number="82"></td>
        <td id="LC82" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L83" class="blob-num js-line-number" data-line-number="83"></td>
        <td id="LC83" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">elif</span> defined ( __TASKING__ )</td>
      </tr>
      <tr>
        <td id="L84" class="blob-num js-line-number" data-line-number="84"></td>
        <td id="LC84" class="blob-code blob-code-inner js-file-line">  #<span class="pl-k">define</span> <span class="pl-en">__ASM</span>            __asm                                      <span class="pl-c"><span class="pl-c">/*</span>!&lt; asm keyword for TASKING Compiler      <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L85" class="blob-num js-line-number" data-line-number="85"></td>
        <td id="LC85" class="blob-code blob-code-inner js-file-line">  #<span class="pl-k">define</span> <span class="pl-en">__INLINE</span>         <span class="pl-k">inline</span>                                     <span class="pl-c"><span class="pl-c">/*</span>!&lt; inline keyword for TASKING Compiler   <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L86" class="blob-num js-line-number" data-line-number="86"></td>
        <td id="LC86" class="blob-code blob-code-inner js-file-line">  #<span class="pl-k">define</span> <span class="pl-en">__STATIC_INLINE</span>  <span class="pl-k">static</span> <span class="pl-k">inline</span></td>
      </tr>
      <tr>
        <td id="L87" class="blob-num js-line-number" data-line-number="87"></td>
        <td id="LC87" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L88" class="blob-num js-line-number" data-line-number="88"></td>
        <td id="LC88" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">endif</span></td>
      </tr>
      <tr>
        <td id="L89" class="blob-num js-line-number" data-line-number="89"></td>
        <td id="LC89" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L90" class="blob-num js-line-number" data-line-number="90"></td>
        <td id="LC90" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* __FPU_USED indicates whether an FPU is used or not. This core does not support an FPU at all</span></td>
      </tr>
      <tr>
        <td id="L91" class="blob-num js-line-number" data-line-number="91"></td>
        <td id="LC91" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L92" class="blob-num js-line-number" data-line-number="92"></td>
        <td id="LC92" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">__FPU_USED</span>       <span class="pl-c1">0</span></td>
      </tr>
      <tr>
        <td id="L93" class="blob-num js-line-number" data-line-number="93"></td>
        <td id="LC93" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L94" class="blob-num js-line-number" data-line-number="94"></td>
        <td id="LC94" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">if</span> defined ( __CC_ARM )</td>
      </tr>
      <tr>
        <td id="L95" class="blob-num js-line-number" data-line-number="95"></td>
        <td id="LC95" class="blob-code blob-code-inner js-file-line">  #<span class="pl-k">if</span> defined __TARGET_FPU_VFP</td>
      </tr>
      <tr>
        <td id="L96" class="blob-num js-line-number" data-line-number="96"></td>
        <td id="LC96" class="blob-code blob-code-inner js-file-line">    #<span class="pl-k">warning</span> &quot;Compiler generates FPU instructions for a device without an FPU (check __FPU_PRESENT)&quot;</td>
      </tr>
      <tr>
        <td id="L97" class="blob-num js-line-number" data-line-number="97"></td>
        <td id="LC97" class="blob-code blob-code-inner js-file-line">  #<span class="pl-k">endif</span></td>
      </tr>
      <tr>
        <td id="L98" class="blob-num js-line-number" data-line-number="98"></td>
        <td id="LC98" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L99" class="blob-num js-line-number" data-line-number="99"></td>
        <td id="LC99" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">elif</span> defined ( __ICCARM__ )</td>
      </tr>
      <tr>
        <td id="L100" class="blob-num js-line-number" data-line-number="100"></td>
        <td id="LC100" class="blob-code blob-code-inner js-file-line">  #<span class="pl-k">if</span> defined __ARMVFP__</td>
      </tr>
      <tr>
        <td id="L101" class="blob-num js-line-number" data-line-number="101"></td>
        <td id="LC101" class="blob-code blob-code-inner js-file-line">    #<span class="pl-k">warning</span> &quot;Compiler generates FPU instructions for a device without an FPU (check __FPU_PRESENT)&quot;</td>
      </tr>
      <tr>
        <td id="L102" class="blob-num js-line-number" data-line-number="102"></td>
        <td id="LC102" class="blob-code blob-code-inner js-file-line">  #<span class="pl-k">endif</span></td>
      </tr>
      <tr>
        <td id="L103" class="blob-num js-line-number" data-line-number="103"></td>
        <td id="LC103" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L104" class="blob-num js-line-number" data-line-number="104"></td>
        <td id="LC104" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">elif</span> defined ( __TMS470__ )</td>
      </tr>
      <tr>
        <td id="L105" class="blob-num js-line-number" data-line-number="105"></td>
        <td id="LC105" class="blob-code blob-code-inner js-file-line">  #<span class="pl-k">if</span> defined __TI__VFP_SUPPORT____</td>
      </tr>
      <tr>
        <td id="L106" class="blob-num js-line-number" data-line-number="106"></td>
        <td id="LC106" class="blob-code blob-code-inner js-file-line">    #<span class="pl-k">warning</span> &quot;Compiler generates FPU instructions for a device without an FPU (check __FPU_PRESENT)&quot;</td>
      </tr>
      <tr>
        <td id="L107" class="blob-num js-line-number" data-line-number="107"></td>
        <td id="LC107" class="blob-code blob-code-inner js-file-line">  #<span class="pl-k">endif</span></td>
      </tr>
      <tr>
        <td id="L108" class="blob-num js-line-number" data-line-number="108"></td>
        <td id="LC108" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L109" class="blob-num js-line-number" data-line-number="109"></td>
        <td id="LC109" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">elif</span> defined ( __GNUC__ )</td>
      </tr>
      <tr>
        <td id="L110" class="blob-num js-line-number" data-line-number="110"></td>
        <td id="LC110" class="blob-code blob-code-inner js-file-line">  #<span class="pl-k">if</span> defined (__VFP_FP__) &amp;&amp; !defined(__SOFTFP__)</td>
      </tr>
      <tr>
        <td id="L111" class="blob-num js-line-number" data-line-number="111"></td>
        <td id="LC111" class="blob-code blob-code-inner js-file-line">    #<span class="pl-k">warning</span> &quot;Compiler generates FPU instructions for a device without an FPU (check __FPU_PRESENT)&quot;</td>
      </tr>
      <tr>
        <td id="L112" class="blob-num js-line-number" data-line-number="112"></td>
        <td id="LC112" class="blob-code blob-code-inner js-file-line">  #<span class="pl-k">endif</span></td>
      </tr>
      <tr>
        <td id="L113" class="blob-num js-line-number" data-line-number="113"></td>
        <td id="LC113" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L114" class="blob-num js-line-number" data-line-number="114"></td>
        <td id="LC114" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">elif</span> defined ( __TASKING__ )</td>
      </tr>
      <tr>
        <td id="L115" class="blob-num js-line-number" data-line-number="115"></td>
        <td id="LC115" class="blob-code blob-code-inner js-file-line">  #<span class="pl-k">if</span> defined __FPU_VFP__</td>
      </tr>
      <tr>
        <td id="L116" class="blob-num js-line-number" data-line-number="116"></td>
        <td id="LC116" class="blob-code blob-code-inner js-file-line">    #<span class="pl-k">error</span> &quot;Compiler generates FPU instructions for a device without an FPU (check __FPU_PRESENT)&quot;</td>
      </tr>
      <tr>
        <td id="L117" class="blob-num js-line-number" data-line-number="117"></td>
        <td id="LC117" class="blob-code blob-code-inner js-file-line">  #<span class="pl-k">endif</span></td>
      </tr>
      <tr>
        <td id="L118" class="blob-num js-line-number" data-line-number="118"></td>
        <td id="LC118" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">endif</span></td>
      </tr>
      <tr>
        <td id="L119" class="blob-num js-line-number" data-line-number="119"></td>
        <td id="LC119" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L120" class="blob-num js-line-number" data-line-number="120"></td>
        <td id="LC120" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">include</span> <span class="pl-s"><span class="pl-pds">&lt;</span>stdint.h<span class="pl-pds">&gt;</span></span>                      <span class="pl-c"><span class="pl-c">/*</span> standard types definitions                      <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L121" class="blob-num js-line-number" data-line-number="121"></td>
        <td id="LC121" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">include</span> <span class="pl-s"><span class="pl-pds">&lt;</span>core_cmInstr.h<span class="pl-pds">&gt;</span></span>                <span class="pl-c"><span class="pl-c">/*</span> Core Instruction Access                         <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L122" class="blob-num js-line-number" data-line-number="122"></td>
        <td id="LC122" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">include</span> <span class="pl-s"><span class="pl-pds">&lt;</span>core_cmFunc.h<span class="pl-pds">&gt;</span></span>                 <span class="pl-c"><span class="pl-c">/*</span> Core Function Access                            <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L123" class="blob-num js-line-number" data-line-number="123"></td>
        <td id="LC123" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L124" class="blob-num js-line-number" data-line-number="124"></td>
        <td id="LC124" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">endif</span> <span class="pl-c"><span class="pl-c">/*</span> __CORE_CM3_H_GENERIC <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L125" class="blob-num js-line-number" data-line-number="125"></td>
        <td id="LC125" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L126" class="blob-num js-line-number" data-line-number="126"></td>
        <td id="LC126" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">ifndef</span> __CMSIS_GENERIC</td>
      </tr>
      <tr>
        <td id="L127" class="blob-num js-line-number" data-line-number="127"></td>
        <td id="LC127" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L128" class="blob-num js-line-number" data-line-number="128"></td>
        <td id="LC128" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">ifndef</span> __CORE_CM3_H_DEPENDANT</td>
      </tr>
      <tr>
        <td id="L129" class="blob-num js-line-number" data-line-number="129"></td>
        <td id="LC129" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">__CORE_CM3_H_DEPENDANT</span></td>
      </tr>
      <tr>
        <td id="L130" class="blob-num js-line-number" data-line-number="130"></td>
        <td id="LC130" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L131" class="blob-num js-line-number" data-line-number="131"></td>
        <td id="LC131" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> check device defines and use defaults <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L132" class="blob-num js-line-number" data-line-number="132"></td>
        <td id="LC132" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">if</span> defined __CHECK_DEVICE_DEFINES</td>
      </tr>
      <tr>
        <td id="L133" class="blob-num js-line-number" data-line-number="133"></td>
        <td id="LC133" class="blob-code blob-code-inner js-file-line">  #<span class="pl-k">ifndef</span> __CM3_REV</td>
      </tr>
      <tr>
        <td id="L134" class="blob-num js-line-number" data-line-number="134"></td>
        <td id="LC134" class="blob-code blob-code-inner js-file-line">    #<span class="pl-k">define</span> <span class="pl-en">__CM3_REV</span>               0x0200</td>
      </tr>
      <tr>
        <td id="L135" class="blob-num js-line-number" data-line-number="135"></td>
        <td id="LC135" class="blob-code blob-code-inner js-file-line">    #<span class="pl-k">warning</span> &quot;__CM3_REV not defined in device header file; using default!&quot;</td>
      </tr>
      <tr>
        <td id="L136" class="blob-num js-line-number" data-line-number="136"></td>
        <td id="LC136" class="blob-code blob-code-inner js-file-line">  #<span class="pl-k">endif</span></td>
      </tr>
      <tr>
        <td id="L137" class="blob-num js-line-number" data-line-number="137"></td>
        <td id="LC137" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L138" class="blob-num js-line-number" data-line-number="138"></td>
        <td id="LC138" class="blob-code blob-code-inner js-file-line">  #<span class="pl-k">ifndef</span> __MPU_PRESENT</td>
      </tr>
      <tr>
        <td id="L139" class="blob-num js-line-number" data-line-number="139"></td>
        <td id="LC139" class="blob-code blob-code-inner js-file-line">    #<span class="pl-k">define</span> <span class="pl-en">__MPU_PRESENT</span>             <span class="pl-c1">0</span></td>
      </tr>
      <tr>
        <td id="L140" class="blob-num js-line-number" data-line-number="140"></td>
        <td id="LC140" class="blob-code blob-code-inner js-file-line">    #<span class="pl-k">warning</span> &quot;__MPU_PRESENT not defined in device header file; using default!&quot;</td>
      </tr>
      <tr>
        <td id="L141" class="blob-num js-line-number" data-line-number="141"></td>
        <td id="LC141" class="blob-code blob-code-inner js-file-line">  #<span class="pl-k">endif</span></td>
      </tr>
      <tr>
        <td id="L142" class="blob-num js-line-number" data-line-number="142"></td>
        <td id="LC142" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L143" class="blob-num js-line-number" data-line-number="143"></td>
        <td id="LC143" class="blob-code blob-code-inner js-file-line">  #<span class="pl-k">ifndef</span> __NVIC_PRIO_BITS</td>
      </tr>
      <tr>
        <td id="L144" class="blob-num js-line-number" data-line-number="144"></td>
        <td id="LC144" class="blob-code blob-code-inner js-file-line">    #<span class="pl-k">define</span> <span class="pl-en">__NVIC_PRIO_BITS</span>          <span class="pl-c1">4</span></td>
      </tr>
      <tr>
        <td id="L145" class="blob-num js-line-number" data-line-number="145"></td>
        <td id="LC145" class="blob-code blob-code-inner js-file-line">    #<span class="pl-k">warning</span> &quot;__NVIC_PRIO_BITS not defined in device header file; using default!&quot;</td>
      </tr>
      <tr>
        <td id="L146" class="blob-num js-line-number" data-line-number="146"></td>
        <td id="LC146" class="blob-code blob-code-inner js-file-line">  #<span class="pl-k">endif</span></td>
      </tr>
      <tr>
        <td id="L147" class="blob-num js-line-number" data-line-number="147"></td>
        <td id="LC147" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L148" class="blob-num js-line-number" data-line-number="148"></td>
        <td id="LC148" class="blob-code blob-code-inner js-file-line">  #<span class="pl-k">ifndef</span> __Vendor_SysTickConfig</td>
      </tr>
      <tr>
        <td id="L149" class="blob-num js-line-number" data-line-number="149"></td>
        <td id="LC149" class="blob-code blob-code-inner js-file-line">    #<span class="pl-k">define</span> <span class="pl-en">__Vendor_SysTickConfig</span>    <span class="pl-c1">0</span></td>
      </tr>
      <tr>
        <td id="L150" class="blob-num js-line-number" data-line-number="150"></td>
        <td id="LC150" class="blob-code blob-code-inner js-file-line">    #<span class="pl-k">warning</span> &quot;__Vendor_SysTickConfig not defined in device header file; using default!&quot;</td>
      </tr>
      <tr>
        <td id="L151" class="blob-num js-line-number" data-line-number="151"></td>
        <td id="LC151" class="blob-code blob-code-inner js-file-line">  #<span class="pl-k">endif</span></td>
      </tr>
      <tr>
        <td id="L152" class="blob-num js-line-number" data-line-number="152"></td>
        <td id="LC152" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">endif</span></td>
      </tr>
      <tr>
        <td id="L153" class="blob-num js-line-number" data-line-number="153"></td>
        <td id="LC153" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L154" class="blob-num js-line-number" data-line-number="154"></td>
        <td id="LC154" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> IO definitions (access restrictions to peripheral registers) <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L155" class="blob-num js-line-number" data-line-number="155"></td>
        <td id="LC155" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>*</span></td>
      </tr>
      <tr>
        <td id="L156" class="blob-num js-line-number" data-line-number="156"></td>
        <td id="LC156" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \defgroup CMSIS_glob_defs CMSIS Global Defines</span></td>
      </tr>
      <tr>
        <td id="L157" class="blob-num js-line-number" data-line-number="157"></td>
        <td id="LC157" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L158" class="blob-num js-line-number" data-line-number="158"></td>
        <td id="LC158" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    &lt;strong&gt;IO Type Qualifiers&lt;/strong&gt; are used</span></td>
      </tr>
      <tr>
        <td id="L159" class="blob-num js-line-number" data-line-number="159"></td>
        <td id="LC159" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \li to specify the access to peripheral variables.</span></td>
      </tr>
      <tr>
        <td id="L160" class="blob-num js-line-number" data-line-number="160"></td>
        <td id="LC160" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \li for automatic generation of peripheral register debug information.</span></td>
      </tr>
      <tr>
        <td id="L161" class="blob-num js-line-number" data-line-number="161"></td>
        <td id="LC161" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L162" class="blob-num js-line-number" data-line-number="162"></td>
        <td id="LC162" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">ifdef</span> __cplusplus</td>
      </tr>
      <tr>
        <td id="L163" class="blob-num js-line-number" data-line-number="163"></td>
        <td id="LC163" class="blob-code blob-code-inner js-file-line">  #<span class="pl-k">define</span>   <span class="pl-en">__I</span>     <span class="pl-k">volatile</span>             <span class="pl-c"><span class="pl-c">/*</span>!&lt; Defines &#39;read only&#39; permissions                 <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L164" class="blob-num js-line-number" data-line-number="164"></td>
        <td id="LC164" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">else</span></td>
      </tr>
      <tr>
        <td id="L165" class="blob-num js-line-number" data-line-number="165"></td>
        <td id="LC165" class="blob-code blob-code-inner js-file-line">  #<span class="pl-k">define</span>   <span class="pl-en">__I</span>     <span class="pl-k">volatile</span> <span class="pl-k">const</span>       <span class="pl-c"><span class="pl-c">/*</span>!&lt; Defines &#39;read only&#39; permissions                 <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L166" class="blob-num js-line-number" data-line-number="166"></td>
        <td id="LC166" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">endif</span></td>
      </tr>
      <tr>
        <td id="L167" class="blob-num js-line-number" data-line-number="167"></td>
        <td id="LC167" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span>     <span class="pl-en">__O</span>     <span class="pl-k">volatile</span>             <span class="pl-c"><span class="pl-c">/*</span>!&lt; Defines &#39;write only&#39; permissions                <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L168" class="blob-num js-line-number" data-line-number="168"></td>
        <td id="LC168" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span>     <span class="pl-en">__IO</span>    <span class="pl-k">volatile</span>             <span class="pl-c"><span class="pl-c">/*</span>!&lt; Defines &#39;read / write&#39; permissions              <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L169" class="blob-num js-line-number" data-line-number="169"></td>
        <td id="LC169" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L170" class="blob-num js-line-number" data-line-number="170"></td>
        <td id="LC170" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>@} end of group Cortex_M3 <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L171" class="blob-num js-line-number" data-line-number="171"></td>
        <td id="LC171" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L172" class="blob-num js-line-number" data-line-number="172"></td>
        <td id="LC172" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L173" class="blob-num js-line-number" data-line-number="173"></td>
        <td id="LC173" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L174" class="blob-num js-line-number" data-line-number="174"></td>
        <td id="LC174" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>******************************************************************************</span></td>
      </tr>
      <tr>
        <td id="L175" class="blob-num js-line-number" data-line-number="175"></td>
        <td id="LC175" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> *                 Register Abstraction</span></td>
      </tr>
      <tr>
        <td id="L176" class="blob-num js-line-number" data-line-number="176"></td>
        <td id="LC176" class="blob-code blob-code-inner js-file-line"><span class="pl-c">  Core Register contain:</span></td>
      </tr>
      <tr>
        <td id="L177" class="blob-num js-line-number" data-line-number="177"></td>
        <td id="LC177" class="blob-code blob-code-inner js-file-line"><span class="pl-c">  - Core Register</span></td>
      </tr>
      <tr>
        <td id="L178" class="blob-num js-line-number" data-line-number="178"></td>
        <td id="LC178" class="blob-code blob-code-inner js-file-line"><span class="pl-c">  - Core NVIC Register</span></td>
      </tr>
      <tr>
        <td id="L179" class="blob-num js-line-number" data-line-number="179"></td>
        <td id="LC179" class="blob-code blob-code-inner js-file-line"><span class="pl-c">  - Core SCB Register</span></td>
      </tr>
      <tr>
        <td id="L180" class="blob-num js-line-number" data-line-number="180"></td>
        <td id="LC180" class="blob-code blob-code-inner js-file-line"><span class="pl-c">  - Core SysTick Register</span></td>
      </tr>
      <tr>
        <td id="L181" class="blob-num js-line-number" data-line-number="181"></td>
        <td id="LC181" class="blob-code blob-code-inner js-file-line"><span class="pl-c">  - Core Debug Register</span></td>
      </tr>
      <tr>
        <td id="L182" class="blob-num js-line-number" data-line-number="182"></td>
        <td id="LC182" class="blob-code blob-code-inner js-file-line"><span class="pl-c">  - Core MPU Register</span></td>
      </tr>
      <tr>
        <td id="L183" class="blob-num js-line-number" data-line-number="183"></td>
        <td id="LC183" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> *****************************************************************************<span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L184" class="blob-num js-line-number" data-line-number="184"></td>
        <td id="LC184" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \defgroup CMSIS_core_register Defines and Type Definitions</span></td>
      </tr>
      <tr>
        <td id="L185" class="blob-num js-line-number" data-line-number="185"></td>
        <td id="LC185" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \brief Type definitions and defines for Cortex-M processor based devices.</span></td>
      </tr>
      <tr>
        <td id="L186" class="blob-num js-line-number" data-line-number="186"></td>
        <td id="LC186" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L187" class="blob-num js-line-number" data-line-number="187"></td>
        <td id="LC187" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L188" class="blob-num js-line-number" data-line-number="188"></td>
        <td id="LC188" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \ingroup    CMSIS_core_register</span></td>
      </tr>
      <tr>
        <td id="L189" class="blob-num js-line-number" data-line-number="189"></td>
        <td id="LC189" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \defgroup   CMSIS_CORE  Status and Control Registers</span></td>
      </tr>
      <tr>
        <td id="L190" class="blob-num js-line-number" data-line-number="190"></td>
        <td id="LC190" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \brief  Core Register type definitions.</span></td>
      </tr>
      <tr>
        <td id="L191" class="blob-num js-line-number" data-line-number="191"></td>
        <td id="LC191" class="blob-code blob-code-inner js-file-line"><span class="pl-c">  @{</span></td>
      </tr>
      <tr>
        <td id="L192" class="blob-num js-line-number" data-line-number="192"></td>
        <td id="LC192" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L193" class="blob-num js-line-number" data-line-number="193"></td>
        <td id="LC193" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L194" class="blob-num js-line-number" data-line-number="194"></td>
        <td id="LC194" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \brief  Union type to access the Application Program Status Register (APSR).</span></td>
      </tr>
      <tr>
        <td id="L195" class="blob-num js-line-number" data-line-number="195"></td>
        <td id="LC195" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L196" class="blob-num js-line-number" data-line-number="196"></td>
        <td id="LC196" class="blob-code blob-code-inner js-file-line"><span class="pl-k">typedef</span> <span class="pl-k">union</span></td>
      </tr>
      <tr>
        <td id="L197" class="blob-num js-line-number" data-line-number="197"></td>
        <td id="LC197" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L198" class="blob-num js-line-number" data-line-number="198"></td>
        <td id="LC198" class="blob-code blob-code-inner js-file-line">  <span class="pl-k">struct</span></td>
      </tr>
      <tr>
        <td id="L199" class="blob-num js-line-number" data-line-number="199"></td>
        <td id="LC199" class="blob-code blob-code-inner js-file-line">  {</td>
      </tr>
      <tr>
        <td id="L200" class="blob-num js-line-number" data-line-number="200"></td>
        <td id="LC200" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">if</span> (__CORTEX_M != 0x04)</td>
      </tr>
      <tr>
        <td id="L201" class="blob-num js-line-number" data-line-number="201"></td>
        <td id="LC201" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">uint32_t</span> _reserved0:<span class="pl-c1">27</span>;              <span class="pl-c"><span class="pl-c">/*</span>!&lt; bit:  0..26  Reserved                           <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L202" class="blob-num js-line-number" data-line-number="202"></td>
        <td id="LC202" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">else</span></td>
      </tr>
      <tr>
        <td id="L203" class="blob-num js-line-number" data-line-number="203"></td>
        <td id="LC203" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">uint32_t</span> _reserved0:<span class="pl-c1">16</span>;              <span class="pl-c"><span class="pl-c">/*</span>!&lt; bit:  0..15  Reserved                           <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L204" class="blob-num js-line-number" data-line-number="204"></td>
        <td id="LC204" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">uint32_t</span> GE:<span class="pl-c1">4</span>;                       <span class="pl-c"><span class="pl-c">/*</span>!&lt; bit: 16..19  Greater than or Equal flags        <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L205" class="blob-num js-line-number" data-line-number="205"></td>
        <td id="LC205" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">uint32_t</span> _reserved1:<span class="pl-c1">7</span>;               <span class="pl-c"><span class="pl-c">/*</span>!&lt; bit: 20..26  Reserved                           <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L206" class="blob-num js-line-number" data-line-number="206"></td>
        <td id="LC206" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">endif</span></td>
      </tr>
      <tr>
        <td id="L207" class="blob-num js-line-number" data-line-number="207"></td>
        <td id="LC207" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">uint32_t</span> Q:<span class="pl-c1">1</span>;                        <span class="pl-c"><span class="pl-c">/*</span>!&lt; bit:     27  Saturation condition flag          <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L208" class="blob-num js-line-number" data-line-number="208"></td>
        <td id="LC208" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">uint32_t</span> V:<span class="pl-c1">1</span>;                        <span class="pl-c"><span class="pl-c">/*</span>!&lt; bit:     28  Overflow condition code flag       <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L209" class="blob-num js-line-number" data-line-number="209"></td>
        <td id="LC209" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">uint32_t</span> C:<span class="pl-c1">1</span>;                        <span class="pl-c"><span class="pl-c">/*</span>!&lt; bit:     29  Carry condition code flag          <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L210" class="blob-num js-line-number" data-line-number="210"></td>
        <td id="LC210" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">uint32_t</span> Z:<span class="pl-c1">1</span>;                        <span class="pl-c"><span class="pl-c">/*</span>!&lt; bit:     30  Zero condition code flag           <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L211" class="blob-num js-line-number" data-line-number="211"></td>
        <td id="LC211" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">uint32_t</span> N:<span class="pl-c1">1</span>;                        <span class="pl-c"><span class="pl-c">/*</span>!&lt; bit:     31  Negative condition code flag       <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L212" class="blob-num js-line-number" data-line-number="212"></td>
        <td id="LC212" class="blob-code blob-code-inner js-file-line">  } b;                                   <span class="pl-c"><span class="pl-c">/*</span>!&lt; Structure used for bit  access                  <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L213" class="blob-num js-line-number" data-line-number="213"></td>
        <td id="LC213" class="blob-code blob-code-inner js-file-line">  <span class="pl-c1">uint32_t</span> w;                            <span class="pl-c"><span class="pl-c">/*</span>!&lt; Type      used for word access                  <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L214" class="blob-num js-line-number" data-line-number="214"></td>
        <td id="LC214" class="blob-code blob-code-inner js-file-line">} APSR_Type;</td>
      </tr>
      <tr>
        <td id="L215" class="blob-num js-line-number" data-line-number="215"></td>
        <td id="LC215" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L216" class="blob-num js-line-number" data-line-number="216"></td>
        <td id="LC216" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L217" class="blob-num js-line-number" data-line-number="217"></td>
        <td id="LC217" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \brief  Union type to access the Interrupt Program Status Register (IPSR).</span></td>
      </tr>
      <tr>
        <td id="L218" class="blob-num js-line-number" data-line-number="218"></td>
        <td id="LC218" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L219" class="blob-num js-line-number" data-line-number="219"></td>
        <td id="LC219" class="blob-code blob-code-inner js-file-line"><span class="pl-k">typedef</span> <span class="pl-k">union</span></td>
      </tr>
      <tr>
        <td id="L220" class="blob-num js-line-number" data-line-number="220"></td>
        <td id="LC220" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L221" class="blob-num js-line-number" data-line-number="221"></td>
        <td id="LC221" class="blob-code blob-code-inner js-file-line">  <span class="pl-k">struct</span></td>
      </tr>
      <tr>
        <td id="L222" class="blob-num js-line-number" data-line-number="222"></td>
        <td id="LC222" class="blob-code blob-code-inner js-file-line">  {</td>
      </tr>
      <tr>
        <td id="L223" class="blob-num js-line-number" data-line-number="223"></td>
        <td id="LC223" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">uint32_t</span> ISR:<span class="pl-c1">9</span>;                      <span class="pl-c"><span class="pl-c">/*</span>!&lt; bit:  0.. 8  Exception number                   <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L224" class="blob-num js-line-number" data-line-number="224"></td>
        <td id="LC224" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">uint32_t</span> _reserved0:<span class="pl-c1">23</span>;              <span class="pl-c"><span class="pl-c">/*</span>!&lt; bit:  9..31  Reserved                           <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L225" class="blob-num js-line-number" data-line-number="225"></td>
        <td id="LC225" class="blob-code blob-code-inner js-file-line">  } b;                                   <span class="pl-c"><span class="pl-c">/*</span>!&lt; Structure used for bit  access                  <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L226" class="blob-num js-line-number" data-line-number="226"></td>
        <td id="LC226" class="blob-code blob-code-inner js-file-line">  <span class="pl-c1">uint32_t</span> w;                            <span class="pl-c"><span class="pl-c">/*</span>!&lt; Type      used for word access                  <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L227" class="blob-num js-line-number" data-line-number="227"></td>
        <td id="LC227" class="blob-code blob-code-inner js-file-line">} IPSR_Type;</td>
      </tr>
      <tr>
        <td id="L228" class="blob-num js-line-number" data-line-number="228"></td>
        <td id="LC228" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L229" class="blob-num js-line-number" data-line-number="229"></td>
        <td id="LC229" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L230" class="blob-num js-line-number" data-line-number="230"></td>
        <td id="LC230" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \brief  Union type to access the Special-Purpose Program Status Registers (xPSR).</span></td>
      </tr>
      <tr>
        <td id="L231" class="blob-num js-line-number" data-line-number="231"></td>
        <td id="LC231" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L232" class="blob-num js-line-number" data-line-number="232"></td>
        <td id="LC232" class="blob-code blob-code-inner js-file-line"><span class="pl-k">typedef</span> <span class="pl-k">union</span></td>
      </tr>
      <tr>
        <td id="L233" class="blob-num js-line-number" data-line-number="233"></td>
        <td id="LC233" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L234" class="blob-num js-line-number" data-line-number="234"></td>
        <td id="LC234" class="blob-code blob-code-inner js-file-line">  <span class="pl-k">struct</span></td>
      </tr>
      <tr>
        <td id="L235" class="blob-num js-line-number" data-line-number="235"></td>
        <td id="LC235" class="blob-code blob-code-inner js-file-line">  {</td>
      </tr>
      <tr>
        <td id="L236" class="blob-num js-line-number" data-line-number="236"></td>
        <td id="LC236" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">uint32_t</span> ISR:<span class="pl-c1">9</span>;                      <span class="pl-c"><span class="pl-c">/*</span>!&lt; bit:  0.. 8  Exception number                   <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L237" class="blob-num js-line-number" data-line-number="237"></td>
        <td id="LC237" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">if</span> (__CORTEX_M != 0x04)</td>
      </tr>
      <tr>
        <td id="L238" class="blob-num js-line-number" data-line-number="238"></td>
        <td id="LC238" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">uint32_t</span> _reserved0:<span class="pl-c1">15</span>;              <span class="pl-c"><span class="pl-c">/*</span>!&lt; bit:  9..23  Reserved                           <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L239" class="blob-num js-line-number" data-line-number="239"></td>
        <td id="LC239" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">else</span></td>
      </tr>
      <tr>
        <td id="L240" class="blob-num js-line-number" data-line-number="240"></td>
        <td id="LC240" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">uint32_t</span> _reserved0:<span class="pl-c1">7</span>;               <span class="pl-c"><span class="pl-c">/*</span>!&lt; bit:  9..15  Reserved                           <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L241" class="blob-num js-line-number" data-line-number="241"></td>
        <td id="LC241" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">uint32_t</span> GE:<span class="pl-c1">4</span>;                       <span class="pl-c"><span class="pl-c">/*</span>!&lt; bit: 16..19  Greater than or Equal flags        <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L242" class="blob-num js-line-number" data-line-number="242"></td>
        <td id="LC242" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">uint32_t</span> _reserved1:<span class="pl-c1">4</span>;               <span class="pl-c"><span class="pl-c">/*</span>!&lt; bit: 20..23  Reserved                           <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L243" class="blob-num js-line-number" data-line-number="243"></td>
        <td id="LC243" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">endif</span></td>
      </tr>
      <tr>
        <td id="L244" class="blob-num js-line-number" data-line-number="244"></td>
        <td id="LC244" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">uint32_t</span> T:<span class="pl-c1">1</span>;                        <span class="pl-c"><span class="pl-c">/*</span>!&lt; bit:     24  Thumb bit        (read 0)          <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L245" class="blob-num js-line-number" data-line-number="245"></td>
        <td id="LC245" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">uint32_t</span> IT:<span class="pl-c1">2</span>;                       <span class="pl-c"><span class="pl-c">/*</span>!&lt; bit: 25..26  saved IT state   (read 0)          <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L246" class="blob-num js-line-number" data-line-number="246"></td>
        <td id="LC246" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">uint32_t</span> Q:<span class="pl-c1">1</span>;                        <span class="pl-c"><span class="pl-c">/*</span>!&lt; bit:     27  Saturation condition flag          <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L247" class="blob-num js-line-number" data-line-number="247"></td>
        <td id="LC247" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">uint32_t</span> V:<span class="pl-c1">1</span>;                        <span class="pl-c"><span class="pl-c">/*</span>!&lt; bit:     28  Overflow condition code flag       <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L248" class="blob-num js-line-number" data-line-number="248"></td>
        <td id="LC248" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">uint32_t</span> C:<span class="pl-c1">1</span>;                        <span class="pl-c"><span class="pl-c">/*</span>!&lt; bit:     29  Carry condition code flag          <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L249" class="blob-num js-line-number" data-line-number="249"></td>
        <td id="LC249" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">uint32_t</span> Z:<span class="pl-c1">1</span>;                        <span class="pl-c"><span class="pl-c">/*</span>!&lt; bit:     30  Zero condition code flag           <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L250" class="blob-num js-line-number" data-line-number="250"></td>
        <td id="LC250" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">uint32_t</span> N:<span class="pl-c1">1</span>;                        <span class="pl-c"><span class="pl-c">/*</span>!&lt; bit:     31  Negative condition code flag       <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L251" class="blob-num js-line-number" data-line-number="251"></td>
        <td id="LC251" class="blob-code blob-code-inner js-file-line">  } b;                                   <span class="pl-c"><span class="pl-c">/*</span>!&lt; Structure used for bit  access                  <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L252" class="blob-num js-line-number" data-line-number="252"></td>
        <td id="LC252" class="blob-code blob-code-inner js-file-line">  <span class="pl-c1">uint32_t</span> w;                            <span class="pl-c"><span class="pl-c">/*</span>!&lt; Type      used for word access                  <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L253" class="blob-num js-line-number" data-line-number="253"></td>
        <td id="LC253" class="blob-code blob-code-inner js-file-line">} xPSR_Type;</td>
      </tr>
      <tr>
        <td id="L254" class="blob-num js-line-number" data-line-number="254"></td>
        <td id="LC254" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L255" class="blob-num js-line-number" data-line-number="255"></td>
        <td id="LC255" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L256" class="blob-num js-line-number" data-line-number="256"></td>
        <td id="LC256" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \brief  Union type to access the Control Registers (CONTROL).</span></td>
      </tr>
      <tr>
        <td id="L257" class="blob-num js-line-number" data-line-number="257"></td>
        <td id="LC257" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L258" class="blob-num js-line-number" data-line-number="258"></td>
        <td id="LC258" class="blob-code blob-code-inner js-file-line"><span class="pl-k">typedef</span> <span class="pl-k">union</span></td>
      </tr>
      <tr>
        <td id="L259" class="blob-num js-line-number" data-line-number="259"></td>
        <td id="LC259" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L260" class="blob-num js-line-number" data-line-number="260"></td>
        <td id="LC260" class="blob-code blob-code-inner js-file-line">  <span class="pl-k">struct</span></td>
      </tr>
      <tr>
        <td id="L261" class="blob-num js-line-number" data-line-number="261"></td>
        <td id="LC261" class="blob-code blob-code-inner js-file-line">  {</td>
      </tr>
      <tr>
        <td id="L262" class="blob-num js-line-number" data-line-number="262"></td>
        <td id="LC262" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">uint32_t</span> nPRIV:<span class="pl-c1">1</span>;                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; bit:      0  Execution privilege in Thread mode <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L263" class="blob-num js-line-number" data-line-number="263"></td>
        <td id="LC263" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">uint32_t</span> SPSEL:<span class="pl-c1">1</span>;                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; bit:      1  Stack to be used                   <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L264" class="blob-num js-line-number" data-line-number="264"></td>
        <td id="LC264" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">uint32_t</span> FPCA:<span class="pl-c1">1</span>;                     <span class="pl-c"><span class="pl-c">/*</span>!&lt; bit:      2  FP extension active flag           <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L265" class="blob-num js-line-number" data-line-number="265"></td>
        <td id="LC265" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">uint32_t</span> _reserved0:<span class="pl-c1">29</span>;              <span class="pl-c"><span class="pl-c">/*</span>!&lt; bit:  3..31  Reserved                           <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L266" class="blob-num js-line-number" data-line-number="266"></td>
        <td id="LC266" class="blob-code blob-code-inner js-file-line">  } b;                                   <span class="pl-c"><span class="pl-c">/*</span>!&lt; Structure used for bit  access                  <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L267" class="blob-num js-line-number" data-line-number="267"></td>
        <td id="LC267" class="blob-code blob-code-inner js-file-line">  <span class="pl-c1">uint32_t</span> w;                            <span class="pl-c"><span class="pl-c">/*</span>!&lt; Type      used for word access                  <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L268" class="blob-num js-line-number" data-line-number="268"></td>
        <td id="LC268" class="blob-code blob-code-inner js-file-line">} CONTROL_Type;</td>
      </tr>
      <tr>
        <td id="L269" class="blob-num js-line-number" data-line-number="269"></td>
        <td id="LC269" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L270" class="blob-num js-line-number" data-line-number="270"></td>
        <td id="LC270" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>@} end of group CMSIS_CORE <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L271" class="blob-num js-line-number" data-line-number="271"></td>
        <td id="LC271" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L272" class="blob-num js-line-number" data-line-number="272"></td>
        <td id="LC272" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L273" class="blob-num js-line-number" data-line-number="273"></td>
        <td id="LC273" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \ingroup    CMSIS_core_register</span></td>
      </tr>
      <tr>
        <td id="L274" class="blob-num js-line-number" data-line-number="274"></td>
        <td id="LC274" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \defgroup   CMSIS_NVIC  Nested Vectored Interrupt Controller (NVIC)</span></td>
      </tr>
      <tr>
        <td id="L275" class="blob-num js-line-number" data-line-number="275"></td>
        <td id="LC275" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \brief      Type definitions for the NVIC Registers</span></td>
      </tr>
      <tr>
        <td id="L276" class="blob-num js-line-number" data-line-number="276"></td>
        <td id="LC276" class="blob-code blob-code-inner js-file-line"><span class="pl-c">  @{</span></td>
      </tr>
      <tr>
        <td id="L277" class="blob-num js-line-number" data-line-number="277"></td>
        <td id="LC277" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L278" class="blob-num js-line-number" data-line-number="278"></td>
        <td id="LC278" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L279" class="blob-num js-line-number" data-line-number="279"></td>
        <td id="LC279" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \brief  Structure type to access the Nested Vectored Interrupt Controller (NVIC).</span></td>
      </tr>
      <tr>
        <td id="L280" class="blob-num js-line-number" data-line-number="280"></td>
        <td id="LC280" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L281" class="blob-num js-line-number" data-line-number="281"></td>
        <td id="LC281" class="blob-code blob-code-inner js-file-line"><span class="pl-k">typedef</span> <span class="pl-k">struct</span></td>
      </tr>
      <tr>
        <td id="L282" class="blob-num js-line-number" data-line-number="282"></td>
        <td id="LC282" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L283" class="blob-num js-line-number" data-line-number="283"></td>
        <td id="LC283" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> ISER[<span class="pl-c1">8</span>];                 <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x000 (R/W)  Interrupt Set Enable Register           <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L284" class="blob-num js-line-number" data-line-number="284"></td>
        <td id="LC284" class="blob-code blob-code-inner js-file-line">       <span class="pl-c1">uint32_t</span> RESERVED0[<span class="pl-c1">24</span>];</td>
      </tr>
      <tr>
        <td id="L285" class="blob-num js-line-number" data-line-number="285"></td>
        <td id="LC285" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> ICER[<span class="pl-c1">8</span>];                 <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x080 (R/W)  Interrupt Clear Enable Register         <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L286" class="blob-num js-line-number" data-line-number="286"></td>
        <td id="LC286" class="blob-code blob-code-inner js-file-line">       <span class="pl-c1">uint32_t</span> RSERVED1[<span class="pl-c1">24</span>];</td>
      </tr>
      <tr>
        <td id="L287" class="blob-num js-line-number" data-line-number="287"></td>
        <td id="LC287" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> ISPR[<span class="pl-c1">8</span>];                 <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x100 (R/W)  Interrupt Set Pending Register          <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L288" class="blob-num js-line-number" data-line-number="288"></td>
        <td id="LC288" class="blob-code blob-code-inner js-file-line">       <span class="pl-c1">uint32_t</span> RESERVED2[<span class="pl-c1">24</span>];</td>
      </tr>
      <tr>
        <td id="L289" class="blob-num js-line-number" data-line-number="289"></td>
        <td id="LC289" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> ICPR[<span class="pl-c1">8</span>];                 <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x180 (R/W)  Interrupt Clear Pending Register        <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L290" class="blob-num js-line-number" data-line-number="290"></td>
        <td id="LC290" class="blob-code blob-code-inner js-file-line">       <span class="pl-c1">uint32_t</span> RESERVED3[<span class="pl-c1">24</span>];</td>
      </tr>
      <tr>
        <td id="L291" class="blob-num js-line-number" data-line-number="291"></td>
        <td id="LC291" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> IABR[<span class="pl-c1">8</span>];                 <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x200 (R/W)  Interrupt Active bit Register           <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L292" class="blob-num js-line-number" data-line-number="292"></td>
        <td id="LC292" class="blob-code blob-code-inner js-file-line">       <span class="pl-c1">uint32_t</span> RESERVED4[<span class="pl-c1">56</span>];</td>
      </tr>
      <tr>
        <td id="L293" class="blob-num js-line-number" data-line-number="293"></td>
        <td id="LC293" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint8_t</span>  IP[<span class="pl-c1">240</span>];                 <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x300 (R/W)  Interrupt Priority Register (8Bit wide) <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L294" class="blob-num js-line-number" data-line-number="294"></td>
        <td id="LC294" class="blob-code blob-code-inner js-file-line">       <span class="pl-c1">uint32_t</span> RESERVED5[<span class="pl-c1">644</span>];</td>
      </tr>
      <tr>
        <td id="L295" class="blob-num js-line-number" data-line-number="295"></td>
        <td id="LC295" class="blob-code blob-code-inner js-file-line">  __O  <span class="pl-c1">uint32_t</span> STIR;                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0xE00 ( /W)  Software Trigger Interrupt Register     <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L296" class="blob-num js-line-number" data-line-number="296"></td>
        <td id="LC296" class="blob-code blob-code-inner js-file-line">}  NVIC_Type;</td>
      </tr>
      <tr>
        <td id="L297" class="blob-num js-line-number" data-line-number="297"></td>
        <td id="LC297" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L298" class="blob-num js-line-number" data-line-number="298"></td>
        <td id="LC298" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> Software Triggered Interrupt Register Definitions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L299" class="blob-num js-line-number" data-line-number="299"></td>
        <td id="LC299" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">NVIC_STIR_INTID_Pos</span>                 <span class="pl-c1">0</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; STIR: INTLINESNUM Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L300" class="blob-num js-line-number" data-line-number="300"></td>
        <td id="LC300" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">NVIC_STIR_INTID_Msk</span>                (0x1FFUL &lt;&lt; NVIC_STIR_INTID_Pos)            <span class="pl-c"><span class="pl-c">/*</span>!&lt; STIR: INTLINESNUM Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L301" class="blob-num js-line-number" data-line-number="301"></td>
        <td id="LC301" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L302" class="blob-num js-line-number" data-line-number="302"></td>
        <td id="LC302" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>@} end of group CMSIS_NVIC <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L303" class="blob-num js-line-number" data-line-number="303"></td>
        <td id="LC303" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L304" class="blob-num js-line-number" data-line-number="304"></td>
        <td id="LC304" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L305" class="blob-num js-line-number" data-line-number="305"></td>
        <td id="LC305" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \ingroup  CMSIS_core_register</span></td>
      </tr>
      <tr>
        <td id="L306" class="blob-num js-line-number" data-line-number="306"></td>
        <td id="LC306" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \defgroup CMSIS_SCB     System Control Block (SCB)</span></td>
      </tr>
      <tr>
        <td id="L307" class="blob-num js-line-number" data-line-number="307"></td>
        <td id="LC307" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \brief      Type definitions for the System Control Block Registers</span></td>
      </tr>
      <tr>
        <td id="L308" class="blob-num js-line-number" data-line-number="308"></td>
        <td id="LC308" class="blob-code blob-code-inner js-file-line"><span class="pl-c">  @{</span></td>
      </tr>
      <tr>
        <td id="L309" class="blob-num js-line-number" data-line-number="309"></td>
        <td id="LC309" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L310" class="blob-num js-line-number" data-line-number="310"></td>
        <td id="LC310" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L311" class="blob-num js-line-number" data-line-number="311"></td>
        <td id="LC311" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \brief  Structure type to access the System Control Block (SCB).</span></td>
      </tr>
      <tr>
        <td id="L312" class="blob-num js-line-number" data-line-number="312"></td>
        <td id="LC312" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L313" class="blob-num js-line-number" data-line-number="313"></td>
        <td id="LC313" class="blob-code blob-code-inner js-file-line"><span class="pl-k">typedef</span> <span class="pl-k">struct</span></td>
      </tr>
      <tr>
        <td id="L314" class="blob-num js-line-number" data-line-number="314"></td>
        <td id="LC314" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L315" class="blob-num js-line-number" data-line-number="315"></td>
        <td id="LC315" class="blob-code blob-code-inner js-file-line">  __I  <span class="pl-c1">uint32_t</span> CPUID;                   <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x000 (R/ )  CPUID Base Register                                   <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L316" class="blob-num js-line-number" data-line-number="316"></td>
        <td id="LC316" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> ICSR;                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x004 (R/W)  Interrupt Control and State Register                  <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L317" class="blob-num js-line-number" data-line-number="317"></td>
        <td id="LC317" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> VTOR;                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x008 (R/W)  Vector Table Offset Register                          <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L318" class="blob-num js-line-number" data-line-number="318"></td>
        <td id="LC318" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> AIRCR;                   <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register      <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L319" class="blob-num js-line-number" data-line-number="319"></td>
        <td id="LC319" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> SCR;                     <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x010 (R/W)  System Control Register                               <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L320" class="blob-num js-line-number" data-line-number="320"></td>
        <td id="LC320" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> CCR;                     <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x014 (R/W)  Configuration Control Register                        <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L321" class="blob-num js-line-number" data-line-number="321"></td>
        <td id="LC321" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint8_t</span>  SHP[<span class="pl-c1">12</span>];                 <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15) <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L322" class="blob-num js-line-number" data-line-number="322"></td>
        <td id="LC322" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> SHCSR;                   <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x024 (R/W)  System Handler Control and State Register             <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L323" class="blob-num js-line-number" data-line-number="323"></td>
        <td id="LC323" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> CFSR;                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x028 (R/W)  Configurable Fault Status Register                    <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L324" class="blob-num js-line-number" data-line-number="324"></td>
        <td id="LC324" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> HFSR;                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x02C (R/W)  HardFault Status Register                             <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L325" class="blob-num js-line-number" data-line-number="325"></td>
        <td id="LC325" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> DFSR;                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x030 (R/W)  Debug Fault Status Register                           <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L326" class="blob-num js-line-number" data-line-number="326"></td>
        <td id="LC326" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> MMFAR;                   <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x034 (R/W)  MemManage Fault Address Register                      <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L327" class="blob-num js-line-number" data-line-number="327"></td>
        <td id="LC327" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> BFAR;                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x038 (R/W)  BusFault Address Register                             <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L328" class="blob-num js-line-number" data-line-number="328"></td>
        <td id="LC328" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> AFSR;                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x03C (R/W)  Auxiliary Fault Status Register                       <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L329" class="blob-num js-line-number" data-line-number="329"></td>
        <td id="LC329" class="blob-code blob-code-inner js-file-line">  __I  <span class="pl-c1">uint32_t</span> PFR[<span class="pl-c1">2</span>];                  <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x040 (R/ )  Processor Feature Register                            <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L330" class="blob-num js-line-number" data-line-number="330"></td>
        <td id="LC330" class="blob-code blob-code-inner js-file-line">  __I  <span class="pl-c1">uint32_t</span> DFR;                     <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x048 (R/ )  Debug Feature Register                                <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L331" class="blob-num js-line-number" data-line-number="331"></td>
        <td id="LC331" class="blob-code blob-code-inner js-file-line">  __I  <span class="pl-c1">uint32_t</span> ADR;                     <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x04C (R/ )  Auxiliary Feature Register                            <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L332" class="blob-num js-line-number" data-line-number="332"></td>
        <td id="LC332" class="blob-code blob-code-inner js-file-line">  __I  <span class="pl-c1">uint32_t</span> MMFR[<span class="pl-c1">4</span>];                 <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x050 (R/ )  Memory Model Feature Register                         <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L333" class="blob-num js-line-number" data-line-number="333"></td>
        <td id="LC333" class="blob-code blob-code-inner js-file-line">  __I  <span class="pl-c1">uint32_t</span> ISAR[<span class="pl-c1">5</span>];                 <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x060 (R/ )  Instruction Set Attributes Register                   <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L334" class="blob-num js-line-number" data-line-number="334"></td>
        <td id="LC334" class="blob-code blob-code-inner js-file-line">       <span class="pl-c1">uint32_t</span> RESERVED0[<span class="pl-c1">5</span>];</td>
      </tr>
      <tr>
        <td id="L335" class="blob-num js-line-number" data-line-number="335"></td>
        <td id="LC335" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> CPACR;                   <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x088 (R/W)  Coprocessor Access Control Register                   <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L336" class="blob-num js-line-number" data-line-number="336"></td>
        <td id="LC336" class="blob-code blob-code-inner js-file-line">} SCB_Type;</td>
      </tr>
      <tr>
        <td id="L337" class="blob-num js-line-number" data-line-number="337"></td>
        <td id="LC337" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L338" class="blob-num js-line-number" data-line-number="338"></td>
        <td id="LC338" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> SCB CPUID Register Definitions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L339" class="blob-num js-line-number" data-line-number="339"></td>
        <td id="LC339" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_CPUID_IMPLEMENTER_Pos</span>          <span class="pl-c1">24</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB CPUID: IMPLEMENTER Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L340" class="blob-num js-line-number" data-line-number="340"></td>
        <td id="LC340" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_CPUID_IMPLEMENTER_Msk</span>          (0xFFUL &lt;&lt; SCB_CPUID_IMPLEMENTER_Pos)          <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB CPUID: IMPLEMENTER Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L341" class="blob-num js-line-number" data-line-number="341"></td>
        <td id="LC341" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L342" class="blob-num js-line-number" data-line-number="342"></td>
        <td id="LC342" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_CPUID_VARIANT_Pos</span>              <span class="pl-c1">20</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB CPUID: VARIANT Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L343" class="blob-num js-line-number" data-line-number="343"></td>
        <td id="LC343" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_CPUID_VARIANT_Msk</span>              (0xFUL &lt;&lt; SCB_CPUID_VARIANT_Pos)               <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB CPUID: VARIANT Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L344" class="blob-num js-line-number" data-line-number="344"></td>
        <td id="LC344" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L345" class="blob-num js-line-number" data-line-number="345"></td>
        <td id="LC345" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_CPUID_ARCHITECTURE_Pos</span>         <span class="pl-c1">16</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB CPUID: ARCHITECTURE Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L346" class="blob-num js-line-number" data-line-number="346"></td>
        <td id="LC346" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_CPUID_ARCHITECTURE_Msk</span>         (0xFUL &lt;&lt; SCB_CPUID_ARCHITECTURE_Pos)          <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB CPUID: ARCHITECTURE Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L347" class="blob-num js-line-number" data-line-number="347"></td>
        <td id="LC347" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L348" class="blob-num js-line-number" data-line-number="348"></td>
        <td id="LC348" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_CPUID_PARTNO_Pos</span>                <span class="pl-c1">4</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB CPUID: PARTNO Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L349" class="blob-num js-line-number" data-line-number="349"></td>
        <td id="LC349" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_CPUID_PARTNO_Msk</span>               (0xFFFUL &lt;&lt; SCB_CPUID_PARTNO_Pos)              <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB CPUID: PARTNO Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L350" class="blob-num js-line-number" data-line-number="350"></td>
        <td id="LC350" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L351" class="blob-num js-line-number" data-line-number="351"></td>
        <td id="LC351" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_CPUID_REVISION_Pos</span>              <span class="pl-c1">0</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB CPUID: REVISION Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L352" class="blob-num js-line-number" data-line-number="352"></td>
        <td id="LC352" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_CPUID_REVISION_Msk</span>             (0xFUL &lt;&lt; SCB_CPUID_REVISION_Pos)              <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB CPUID: REVISION Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L353" class="blob-num js-line-number" data-line-number="353"></td>
        <td id="LC353" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L354" class="blob-num js-line-number" data-line-number="354"></td>
        <td id="LC354" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> SCB Interrupt Control State Register Definitions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L355" class="blob-num js-line-number" data-line-number="355"></td>
        <td id="LC355" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_ICSR_NMIPENDSET_Pos</span>            <span class="pl-c1">31</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB ICSR: NMIPENDSET Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L356" class="blob-num js-line-number" data-line-number="356"></td>
        <td id="LC356" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_ICSR_NMIPENDSET_Msk</span>            (<span class="pl-c1">1UL</span> &lt;&lt; SCB_ICSR_NMIPENDSET_Pos)               <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB ICSR: NMIPENDSET Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L357" class="blob-num js-line-number" data-line-number="357"></td>
        <td id="LC357" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L358" class="blob-num js-line-number" data-line-number="358"></td>
        <td id="LC358" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_ICSR_PENDSVSET_Pos</span>             <span class="pl-c1">28</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB ICSR: PENDSVSET Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L359" class="blob-num js-line-number" data-line-number="359"></td>
        <td id="LC359" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_ICSR_PENDSVSET_Msk</span>             (<span class="pl-c1">1UL</span> &lt;&lt; SCB_ICSR_PENDSVSET_Pos)                <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB ICSR: PENDSVSET Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L360" class="blob-num js-line-number" data-line-number="360"></td>
        <td id="LC360" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L361" class="blob-num js-line-number" data-line-number="361"></td>
        <td id="LC361" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_ICSR_PENDSVCLR_Pos</span>             <span class="pl-c1">27</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB ICSR: PENDSVCLR Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L362" class="blob-num js-line-number" data-line-number="362"></td>
        <td id="LC362" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_ICSR_PENDSVCLR_Msk</span>             (<span class="pl-c1">1UL</span> &lt;&lt; SCB_ICSR_PENDSVCLR_Pos)                <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB ICSR: PENDSVCLR Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L363" class="blob-num js-line-number" data-line-number="363"></td>
        <td id="LC363" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L364" class="blob-num js-line-number" data-line-number="364"></td>
        <td id="LC364" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_ICSR_PENDSTSET_Pos</span>             <span class="pl-c1">26</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB ICSR: PENDSTSET Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L365" class="blob-num js-line-number" data-line-number="365"></td>
        <td id="LC365" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_ICSR_PENDSTSET_Msk</span>             (<span class="pl-c1">1UL</span> &lt;&lt; SCB_ICSR_PENDSTSET_Pos)                <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB ICSR: PENDSTSET Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L366" class="blob-num js-line-number" data-line-number="366"></td>
        <td id="LC366" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L367" class="blob-num js-line-number" data-line-number="367"></td>
        <td id="LC367" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_ICSR_PENDSTCLR_Pos</span>             <span class="pl-c1">25</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB ICSR: PENDSTCLR Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L368" class="blob-num js-line-number" data-line-number="368"></td>
        <td id="LC368" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_ICSR_PENDSTCLR_Msk</span>             (<span class="pl-c1">1UL</span> &lt;&lt; SCB_ICSR_PENDSTCLR_Pos)                <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB ICSR: PENDSTCLR Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L369" class="blob-num js-line-number" data-line-number="369"></td>
        <td id="LC369" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L370" class="blob-num js-line-number" data-line-number="370"></td>
        <td id="LC370" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_ICSR_ISRPREEMPT_Pos</span>            <span class="pl-c1">23</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB ICSR: ISRPREEMPT Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L371" class="blob-num js-line-number" data-line-number="371"></td>
        <td id="LC371" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_ICSR_ISRPREEMPT_Msk</span>            (<span class="pl-c1">1UL</span> &lt;&lt; SCB_ICSR_ISRPREEMPT_Pos)               <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB ICSR: ISRPREEMPT Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L372" class="blob-num js-line-number" data-line-number="372"></td>
        <td id="LC372" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L373" class="blob-num js-line-number" data-line-number="373"></td>
        <td id="LC373" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_ICSR_ISRPENDING_Pos</span>            <span class="pl-c1">22</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB ICSR: ISRPENDING Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L374" class="blob-num js-line-number" data-line-number="374"></td>
        <td id="LC374" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_ICSR_ISRPENDING_Msk</span>            (<span class="pl-c1">1UL</span> &lt;&lt; SCB_ICSR_ISRPENDING_Pos)               <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB ICSR: ISRPENDING Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L375" class="blob-num js-line-number" data-line-number="375"></td>
        <td id="LC375" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L376" class="blob-num js-line-number" data-line-number="376"></td>
        <td id="LC376" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_ICSR_VECTPENDING_Pos</span>           <span class="pl-c1">12</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB ICSR: VECTPENDING Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L377" class="blob-num js-line-number" data-line-number="377"></td>
        <td id="LC377" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_ICSR_VECTPENDING_Msk</span>           (0x1FFUL &lt;&lt; SCB_ICSR_VECTPENDING_Pos)          <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB ICSR: VECTPENDING Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L378" class="blob-num js-line-number" data-line-number="378"></td>
        <td id="LC378" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L379" class="blob-num js-line-number" data-line-number="379"></td>
        <td id="LC379" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_ICSR_RETTOBASE_Pos</span>             <span class="pl-c1">11</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB ICSR: RETTOBASE Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L380" class="blob-num js-line-number" data-line-number="380"></td>
        <td id="LC380" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_ICSR_RETTOBASE_Msk</span>             (<span class="pl-c1">1UL</span> &lt;&lt; SCB_ICSR_RETTOBASE_Pos)                <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB ICSR: RETTOBASE Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L381" class="blob-num js-line-number" data-line-number="381"></td>
        <td id="LC381" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L382" class="blob-num js-line-number" data-line-number="382"></td>
        <td id="LC382" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_ICSR_VECTACTIVE_Pos</span>             <span class="pl-c1">0</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB ICSR: VECTACTIVE Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L383" class="blob-num js-line-number" data-line-number="383"></td>
        <td id="LC383" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_ICSR_VECTACTIVE_Msk</span>            (0x1FFUL &lt;&lt; SCB_ICSR_VECTACTIVE_Pos)           <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB ICSR: VECTACTIVE Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L384" class="blob-num js-line-number" data-line-number="384"></td>
        <td id="LC384" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L385" class="blob-num js-line-number" data-line-number="385"></td>
        <td id="LC385" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> SCB Vector Table Offset Register Definitions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L386" class="blob-num js-line-number" data-line-number="386"></td>
        <td id="LC386" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">if</span> (__CM3_REV &lt; 0x0201)                   <span class="pl-c"><span class="pl-c">/*</span> core r2p1 <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L387" class="blob-num js-line-number" data-line-number="387"></td>
        <td id="LC387" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_VTOR_TBLBASE_Pos</span>               <span class="pl-c1">29</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB VTOR: TBLBASE Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L388" class="blob-num js-line-number" data-line-number="388"></td>
        <td id="LC388" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_VTOR_TBLBASE_Msk</span>               (<span class="pl-c1">1UL</span> &lt;&lt; SCB_VTOR_TBLBASE_Pos)                  <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB VTOR: TBLBASE Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L389" class="blob-num js-line-number" data-line-number="389"></td>
        <td id="LC389" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L390" class="blob-num js-line-number" data-line-number="390"></td>
        <td id="LC390" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_VTOR_TBLOFF_Pos</span>                 <span class="pl-c1">7</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB VTOR: TBLOFF Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L391" class="blob-num js-line-number" data-line-number="391"></td>
        <td id="LC391" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_VTOR_TBLOFF_Msk</span>                (0x3FFFFFUL &lt;&lt; SCB_VTOR_TBLOFF_Pos)            <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB VTOR: TBLOFF Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L392" class="blob-num js-line-number" data-line-number="392"></td>
        <td id="LC392" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">else</span></td>
      </tr>
      <tr>
        <td id="L393" class="blob-num js-line-number" data-line-number="393"></td>
        <td id="LC393" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_VTOR_TBLOFF_Pos</span>                 <span class="pl-c1">7</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB VTOR: TBLOFF Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L394" class="blob-num js-line-number" data-line-number="394"></td>
        <td id="LC394" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_VTOR_TBLOFF_Msk</span>                (0x1FFFFFFUL &lt;&lt; SCB_VTOR_TBLOFF_Pos)           <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB VTOR: TBLOFF Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L395" class="blob-num js-line-number" data-line-number="395"></td>
        <td id="LC395" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">endif</span></td>
      </tr>
      <tr>
        <td id="L396" class="blob-num js-line-number" data-line-number="396"></td>
        <td id="LC396" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L397" class="blob-num js-line-number" data-line-number="397"></td>
        <td id="LC397" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> SCB Application Interrupt and Reset Control Register Definitions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L398" class="blob-num js-line-number" data-line-number="398"></td>
        <td id="LC398" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_AIRCR_VECTKEY_Pos</span>              <span class="pl-c1">16</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB AIRCR: VECTKEY Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L399" class="blob-num js-line-number" data-line-number="399"></td>
        <td id="LC399" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_AIRCR_VECTKEY_Msk</span>              (0xFFFFUL &lt;&lt; SCB_AIRCR_VECTKEY_Pos)            <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB AIRCR: VECTKEY Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L400" class="blob-num js-line-number" data-line-number="400"></td>
        <td id="LC400" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L401" class="blob-num js-line-number" data-line-number="401"></td>
        <td id="LC401" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_AIRCR_VECTKEYSTAT_Pos</span>          <span class="pl-c1">16</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB AIRCR: VECTKEYSTAT Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L402" class="blob-num js-line-number" data-line-number="402"></td>
        <td id="LC402" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_AIRCR_VECTKEYSTAT_Msk</span>          (0xFFFFUL &lt;&lt; SCB_AIRCR_VECTKEYSTAT_Pos)        <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB AIRCR: VECTKEYSTAT Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L403" class="blob-num js-line-number" data-line-number="403"></td>
        <td id="LC403" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L404" class="blob-num js-line-number" data-line-number="404"></td>
        <td id="LC404" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_AIRCR_ENDIANESS_Pos</span>            <span class="pl-c1">15</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB AIRCR: ENDIANESS Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L405" class="blob-num js-line-number" data-line-number="405"></td>
        <td id="LC405" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_AIRCR_ENDIANESS_Msk</span>            (<span class="pl-c1">1UL</span> &lt;&lt; SCB_AIRCR_ENDIANESS_Pos)               <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB AIRCR: ENDIANESS Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L406" class="blob-num js-line-number" data-line-number="406"></td>
        <td id="LC406" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L407" class="blob-num js-line-number" data-line-number="407"></td>
        <td id="LC407" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_AIRCR_PRIGROUP_Pos</span>              <span class="pl-c1">8</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB AIRCR: PRIGROUP Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L408" class="blob-num js-line-number" data-line-number="408"></td>
        <td id="LC408" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_AIRCR_PRIGROUP_Msk</span>             (<span class="pl-c1">7UL</span> &lt;&lt; SCB_AIRCR_PRIGROUP_Pos)                <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB AIRCR: PRIGROUP Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L409" class="blob-num js-line-number" data-line-number="409"></td>
        <td id="LC409" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L410" class="blob-num js-line-number" data-line-number="410"></td>
        <td id="LC410" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_AIRCR_SYSRESETREQ_Pos</span>           <span class="pl-c1">2</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB AIRCR: SYSRESETREQ Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L411" class="blob-num js-line-number" data-line-number="411"></td>
        <td id="LC411" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_AIRCR_SYSRESETREQ_Msk</span>          (<span class="pl-c1">1UL</span> &lt;&lt; SCB_AIRCR_SYSRESETREQ_Pos)             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB AIRCR: SYSRESETREQ Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L412" class="blob-num js-line-number" data-line-number="412"></td>
        <td id="LC412" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L413" class="blob-num js-line-number" data-line-number="413"></td>
        <td id="LC413" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_AIRCR_VECTCLRACTIVE_Pos</span>         <span class="pl-c1">1</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB AIRCR: VECTCLRACTIVE Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L414" class="blob-num js-line-number" data-line-number="414"></td>
        <td id="LC414" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_AIRCR_VECTCLRACTIVE_Msk</span>        (<span class="pl-c1">1UL</span> &lt;&lt; SCB_AIRCR_VECTCLRACTIVE_Pos)           <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB AIRCR: VECTCLRACTIVE Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L415" class="blob-num js-line-number" data-line-number="415"></td>
        <td id="LC415" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L416" class="blob-num js-line-number" data-line-number="416"></td>
        <td id="LC416" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_AIRCR_VECTRESET_Pos</span>             <span class="pl-c1">0</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB AIRCR: VECTRESET Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L417" class="blob-num js-line-number" data-line-number="417"></td>
        <td id="LC417" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_AIRCR_VECTRESET_Msk</span>            (<span class="pl-c1">1UL</span> &lt;&lt; SCB_AIRCR_VECTRESET_Pos)               <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB AIRCR: VECTRESET Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L418" class="blob-num js-line-number" data-line-number="418"></td>
        <td id="LC418" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L419" class="blob-num js-line-number" data-line-number="419"></td>
        <td id="LC419" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> SCB System Control Register Definitions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L420" class="blob-num js-line-number" data-line-number="420"></td>
        <td id="LC420" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_SCR_SEVONPEND_Pos</span>               <span class="pl-c1">4</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB SCR: SEVONPEND Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L421" class="blob-num js-line-number" data-line-number="421"></td>
        <td id="LC421" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_SCR_SEVONPEND_Msk</span>              (<span class="pl-c1">1UL</span> &lt;&lt; SCB_SCR_SEVONPEND_Pos)                 <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB SCR: SEVONPEND Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L422" class="blob-num js-line-number" data-line-number="422"></td>
        <td id="LC422" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L423" class="blob-num js-line-number" data-line-number="423"></td>
        <td id="LC423" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_SCR_SLEEPDEEP_Pos</span>               <span class="pl-c1">2</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB SCR: SLEEPDEEP Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L424" class="blob-num js-line-number" data-line-number="424"></td>
        <td id="LC424" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_SCR_SLEEPDEEP_Msk</span>              (<span class="pl-c1">1UL</span> &lt;&lt; SCB_SCR_SLEEPDEEP_Pos)                 <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB SCR: SLEEPDEEP Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L425" class="blob-num js-line-number" data-line-number="425"></td>
        <td id="LC425" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L426" class="blob-num js-line-number" data-line-number="426"></td>
        <td id="LC426" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_SCR_SLEEPONEXIT_Pos</span>             <span class="pl-c1">1</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB SCR: SLEEPONEXIT Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L427" class="blob-num js-line-number" data-line-number="427"></td>
        <td id="LC427" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_SCR_SLEEPONEXIT_Msk</span>            (<span class="pl-c1">1UL</span> &lt;&lt; SCB_SCR_SLEEPONEXIT_Pos)               <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB SCR: SLEEPONEXIT Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L428" class="blob-num js-line-number" data-line-number="428"></td>
        <td id="LC428" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L429" class="blob-num js-line-number" data-line-number="429"></td>
        <td id="LC429" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> SCB Configuration Control Register Definitions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L430" class="blob-num js-line-number" data-line-number="430"></td>
        <td id="LC430" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_CCR_STKALIGN_Pos</span>                <span class="pl-c1">9</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB CCR: STKALIGN Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L431" class="blob-num js-line-number" data-line-number="431"></td>
        <td id="LC431" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_CCR_STKALIGN_Msk</span>               (<span class="pl-c1">1UL</span> &lt;&lt; SCB_CCR_STKALIGN_Pos)                  <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB CCR: STKALIGN Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L432" class="blob-num js-line-number" data-line-number="432"></td>
        <td id="LC432" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L433" class="blob-num js-line-number" data-line-number="433"></td>
        <td id="LC433" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_CCR_BFHFNMIGN_Pos</span>               <span class="pl-c1">8</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB CCR: BFHFNMIGN Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L434" class="blob-num js-line-number" data-line-number="434"></td>
        <td id="LC434" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_CCR_BFHFNMIGN_Msk</span>              (<span class="pl-c1">1UL</span> &lt;&lt; SCB_CCR_BFHFNMIGN_Pos)                 <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB CCR: BFHFNMIGN Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L435" class="blob-num js-line-number" data-line-number="435"></td>
        <td id="LC435" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L436" class="blob-num js-line-number" data-line-number="436"></td>
        <td id="LC436" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_CCR_DIV_0_TRP_Pos</span>               <span class="pl-c1">4</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB CCR: DIV_0_TRP Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L437" class="blob-num js-line-number" data-line-number="437"></td>
        <td id="LC437" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_CCR_DIV_0_TRP_Msk</span>              (<span class="pl-c1">1UL</span> &lt;&lt; SCB_CCR_DIV_0_TRP_Pos)                 <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB CCR: DIV_0_TRP Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L438" class="blob-num js-line-number" data-line-number="438"></td>
        <td id="LC438" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L439" class="blob-num js-line-number" data-line-number="439"></td>
        <td id="LC439" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_CCR_UNALIGN_TRP_Pos</span>             <span class="pl-c1">3</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB CCR: UNALIGN_TRP Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L440" class="blob-num js-line-number" data-line-number="440"></td>
        <td id="LC440" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_CCR_UNALIGN_TRP_Msk</span>            (<span class="pl-c1">1UL</span> &lt;&lt; SCB_CCR_UNALIGN_TRP_Pos)               <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB CCR: UNALIGN_TRP Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L441" class="blob-num js-line-number" data-line-number="441"></td>
        <td id="LC441" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L442" class="blob-num js-line-number" data-line-number="442"></td>
        <td id="LC442" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_CCR_USERSETMPEND_Pos</span>            <span class="pl-c1">1</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB CCR: USERSETMPEND Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L443" class="blob-num js-line-number" data-line-number="443"></td>
        <td id="LC443" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_CCR_USERSETMPEND_Msk</span>           (<span class="pl-c1">1UL</span> &lt;&lt; SCB_CCR_USERSETMPEND_Pos)              <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB CCR: USERSETMPEND Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L444" class="blob-num js-line-number" data-line-number="444"></td>
        <td id="LC444" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L445" class="blob-num js-line-number" data-line-number="445"></td>
        <td id="LC445" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_CCR_NONBASETHRDENA_Pos</span>          <span class="pl-c1">0</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB CCR: NONBASETHRDENA Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L446" class="blob-num js-line-number" data-line-number="446"></td>
        <td id="LC446" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_CCR_NONBASETHRDENA_Msk</span>         (<span class="pl-c1">1UL</span> &lt;&lt; SCB_CCR_NONBASETHRDENA_Pos)            <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB CCR: NONBASETHRDENA Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L447" class="blob-num js-line-number" data-line-number="447"></td>
        <td id="LC447" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L448" class="blob-num js-line-number" data-line-number="448"></td>
        <td id="LC448" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> SCB System Handler Control and State Register Definitions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L449" class="blob-num js-line-number" data-line-number="449"></td>
        <td id="LC449" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_SHCSR_USGFAULTENA_Pos</span>          <span class="pl-c1">18</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB SHCSR: USGFAULTENA Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L450" class="blob-num js-line-number" data-line-number="450"></td>
        <td id="LC450" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_SHCSR_USGFAULTENA_Msk</span>          (<span class="pl-c1">1UL</span> &lt;&lt; SCB_SHCSR_USGFAULTENA_Pos)             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB SHCSR: USGFAULTENA Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L451" class="blob-num js-line-number" data-line-number="451"></td>
        <td id="LC451" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L452" class="blob-num js-line-number" data-line-number="452"></td>
        <td id="LC452" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_SHCSR_BUSFAULTENA_Pos</span>          <span class="pl-c1">17</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB SHCSR: BUSFAULTENA Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L453" class="blob-num js-line-number" data-line-number="453"></td>
        <td id="LC453" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_SHCSR_BUSFAULTENA_Msk</span>          (<span class="pl-c1">1UL</span> &lt;&lt; SCB_SHCSR_BUSFAULTENA_Pos)             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB SHCSR: BUSFAULTENA Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L454" class="blob-num js-line-number" data-line-number="454"></td>
        <td id="LC454" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L455" class="blob-num js-line-number" data-line-number="455"></td>
        <td id="LC455" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_SHCSR_MEMFAULTENA_Pos</span>          <span class="pl-c1">16</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB SHCSR: MEMFAULTENA Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L456" class="blob-num js-line-number" data-line-number="456"></td>
        <td id="LC456" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_SHCSR_MEMFAULTENA_Msk</span>          (<span class="pl-c1">1UL</span> &lt;&lt; SCB_SHCSR_MEMFAULTENA_Pos)             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB SHCSR: MEMFAULTENA Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L457" class="blob-num js-line-number" data-line-number="457"></td>
        <td id="LC457" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L458" class="blob-num js-line-number" data-line-number="458"></td>
        <td id="LC458" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_SHCSR_SVCALLPENDED_Pos</span>         <span class="pl-c1">15</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB SHCSR: SVCALLPENDED Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L459" class="blob-num js-line-number" data-line-number="459"></td>
        <td id="LC459" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_SHCSR_SVCALLPENDED_Msk</span>         (<span class="pl-c1">1UL</span> &lt;&lt; SCB_SHCSR_SVCALLPENDED_Pos)            <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB SHCSR: SVCALLPENDED Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L460" class="blob-num js-line-number" data-line-number="460"></td>
        <td id="LC460" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L461" class="blob-num js-line-number" data-line-number="461"></td>
        <td id="LC461" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_SHCSR_BUSFAULTPENDED_Pos</span>       <span class="pl-c1">14</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB SHCSR: BUSFAULTPENDED Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L462" class="blob-num js-line-number" data-line-number="462"></td>
        <td id="LC462" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_SHCSR_BUSFAULTPENDED_Msk</span>       (<span class="pl-c1">1UL</span> &lt;&lt; SCB_SHCSR_BUSFAULTPENDED_Pos)          <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB SHCSR: BUSFAULTPENDED Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L463" class="blob-num js-line-number" data-line-number="463"></td>
        <td id="LC463" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L464" class="blob-num js-line-number" data-line-number="464"></td>
        <td id="LC464" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_SHCSR_MEMFAULTPENDED_Pos</span>       <span class="pl-c1">13</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB SHCSR: MEMFAULTPENDED Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L465" class="blob-num js-line-number" data-line-number="465"></td>
        <td id="LC465" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_SHCSR_MEMFAULTPENDED_Msk</span>       (<span class="pl-c1">1UL</span> &lt;&lt; SCB_SHCSR_MEMFAULTPENDED_Pos)          <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB SHCSR: MEMFAULTPENDED Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L466" class="blob-num js-line-number" data-line-number="466"></td>
        <td id="LC466" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L467" class="blob-num js-line-number" data-line-number="467"></td>
        <td id="LC467" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_SHCSR_USGFAULTPENDED_Pos</span>       <span class="pl-c1">12</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB SHCSR: USGFAULTPENDED Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L468" class="blob-num js-line-number" data-line-number="468"></td>
        <td id="LC468" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_SHCSR_USGFAULTPENDED_Msk</span>       (<span class="pl-c1">1UL</span> &lt;&lt; SCB_SHCSR_USGFAULTPENDED_Pos)          <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB SHCSR: USGFAULTPENDED Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L469" class="blob-num js-line-number" data-line-number="469"></td>
        <td id="LC469" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L470" class="blob-num js-line-number" data-line-number="470"></td>
        <td id="LC470" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_SHCSR_SYSTICKACT_Pos</span>           <span class="pl-c1">11</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB SHCSR: SYSTICKACT Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L471" class="blob-num js-line-number" data-line-number="471"></td>
        <td id="LC471" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_SHCSR_SYSTICKACT_Msk</span>           (<span class="pl-c1">1UL</span> &lt;&lt; SCB_SHCSR_SYSTICKACT_Pos)              <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB SHCSR: SYSTICKACT Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L472" class="blob-num js-line-number" data-line-number="472"></td>
        <td id="LC472" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L473" class="blob-num js-line-number" data-line-number="473"></td>
        <td id="LC473" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_SHCSR_PENDSVACT_Pos</span>            <span class="pl-c1">10</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB SHCSR: PENDSVACT Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L474" class="blob-num js-line-number" data-line-number="474"></td>
        <td id="LC474" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_SHCSR_PENDSVACT_Msk</span>            (<span class="pl-c1">1UL</span> &lt;&lt; SCB_SHCSR_PENDSVACT_Pos)               <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB SHCSR: PENDSVACT Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L475" class="blob-num js-line-number" data-line-number="475"></td>
        <td id="LC475" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L476" class="blob-num js-line-number" data-line-number="476"></td>
        <td id="LC476" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_SHCSR_MONITORACT_Pos</span>            <span class="pl-c1">8</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB SHCSR: MONITORACT Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L477" class="blob-num js-line-number" data-line-number="477"></td>
        <td id="LC477" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_SHCSR_MONITORACT_Msk</span>           (<span class="pl-c1">1UL</span> &lt;&lt; SCB_SHCSR_MONITORACT_Pos)              <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB SHCSR: MONITORACT Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L478" class="blob-num js-line-number" data-line-number="478"></td>
        <td id="LC478" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L479" class="blob-num js-line-number" data-line-number="479"></td>
        <td id="LC479" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_SHCSR_SVCALLACT_Pos</span>             <span class="pl-c1">7</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB SHCSR: SVCALLACT Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L480" class="blob-num js-line-number" data-line-number="480"></td>
        <td id="LC480" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_SHCSR_SVCALLACT_Msk</span>            (<span class="pl-c1">1UL</span> &lt;&lt; SCB_SHCSR_SVCALLACT_Pos)               <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB SHCSR: SVCALLACT Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L481" class="blob-num js-line-number" data-line-number="481"></td>
        <td id="LC481" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L482" class="blob-num js-line-number" data-line-number="482"></td>
        <td id="LC482" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_SHCSR_USGFAULTACT_Pos</span>           <span class="pl-c1">3</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB SHCSR: USGFAULTACT Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L483" class="blob-num js-line-number" data-line-number="483"></td>
        <td id="LC483" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_SHCSR_USGFAULTACT_Msk</span>          (<span class="pl-c1">1UL</span> &lt;&lt; SCB_SHCSR_USGFAULTACT_Pos)             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB SHCSR: USGFAULTACT Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L484" class="blob-num js-line-number" data-line-number="484"></td>
        <td id="LC484" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L485" class="blob-num js-line-number" data-line-number="485"></td>
        <td id="LC485" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_SHCSR_BUSFAULTACT_Pos</span>           <span class="pl-c1">1</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB SHCSR: BUSFAULTACT Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L486" class="blob-num js-line-number" data-line-number="486"></td>
        <td id="LC486" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_SHCSR_BUSFAULTACT_Msk</span>          (<span class="pl-c1">1UL</span> &lt;&lt; SCB_SHCSR_BUSFAULTACT_Pos)             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB SHCSR: BUSFAULTACT Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L487" class="blob-num js-line-number" data-line-number="487"></td>
        <td id="LC487" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L488" class="blob-num js-line-number" data-line-number="488"></td>
        <td id="LC488" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_SHCSR_MEMFAULTACT_Pos</span>           <span class="pl-c1">0</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB SHCSR: MEMFAULTACT Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L489" class="blob-num js-line-number" data-line-number="489"></td>
        <td id="LC489" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_SHCSR_MEMFAULTACT_Msk</span>          (<span class="pl-c1">1UL</span> &lt;&lt; SCB_SHCSR_MEMFAULTACT_Pos)             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB SHCSR: MEMFAULTACT Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L490" class="blob-num js-line-number" data-line-number="490"></td>
        <td id="LC490" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L491" class="blob-num js-line-number" data-line-number="491"></td>
        <td id="LC491" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> SCB Configurable Fault Status Registers Definitions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L492" class="blob-num js-line-number" data-line-number="492"></td>
        <td id="LC492" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_CFSR_USGFAULTSR_Pos</span>            <span class="pl-c1">16</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB CFSR: Usage Fault Status Register Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L493" class="blob-num js-line-number" data-line-number="493"></td>
        <td id="LC493" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_CFSR_USGFAULTSR_Msk</span>            (0xFFFFUL &lt;&lt; SCB_CFSR_USGFAULTSR_Pos)          <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB CFSR: Usage Fault Status Register Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L494" class="blob-num js-line-number" data-line-number="494"></td>
        <td id="LC494" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L495" class="blob-num js-line-number" data-line-number="495"></td>
        <td id="LC495" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_CFSR_BUSFAULTSR_Pos</span>             <span class="pl-c1">8</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB CFSR: Bus Fault Status Register Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L496" class="blob-num js-line-number" data-line-number="496"></td>
        <td id="LC496" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_CFSR_BUSFAULTSR_Msk</span>            (0xFFUL &lt;&lt; SCB_CFSR_BUSFAULTSR_Pos)            <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB CFSR: Bus Fault Status Register Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L497" class="blob-num js-line-number" data-line-number="497"></td>
        <td id="LC497" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L498" class="blob-num js-line-number" data-line-number="498"></td>
        <td id="LC498" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_CFSR_MEMFAULTSR_Pos</span>             <span class="pl-c1">0</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB CFSR: Memory Manage Fault Status Register Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L499" class="blob-num js-line-number" data-line-number="499"></td>
        <td id="LC499" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_CFSR_MEMFAULTSR_Msk</span>            (0xFFUL &lt;&lt; SCB_CFSR_MEMFAULTSR_Pos)            <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB CFSR: Memory Manage Fault Status Register Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L500" class="blob-num js-line-number" data-line-number="500"></td>
        <td id="LC500" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L501" class="blob-num js-line-number" data-line-number="501"></td>
        <td id="LC501" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> SCB Hard Fault Status Registers Definitions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L502" class="blob-num js-line-number" data-line-number="502"></td>
        <td id="LC502" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_HFSR_DEBUGEVT_Pos</span>              <span class="pl-c1">31</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB HFSR: DEBUGEVT Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L503" class="blob-num js-line-number" data-line-number="503"></td>
        <td id="LC503" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_HFSR_DEBUGEVT_Msk</span>              (<span class="pl-c1">1UL</span> &lt;&lt; SCB_HFSR_DEBUGEVT_Pos)                 <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB HFSR: DEBUGEVT Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L504" class="blob-num js-line-number" data-line-number="504"></td>
        <td id="LC504" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L505" class="blob-num js-line-number" data-line-number="505"></td>
        <td id="LC505" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_HFSR_FORCED_Pos</span>                <span class="pl-c1">30</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB HFSR: FORCED Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L506" class="blob-num js-line-number" data-line-number="506"></td>
        <td id="LC506" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_HFSR_FORCED_Msk</span>                (<span class="pl-c1">1UL</span> &lt;&lt; SCB_HFSR_FORCED_Pos)                   <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB HFSR: FORCED Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L507" class="blob-num js-line-number" data-line-number="507"></td>
        <td id="LC507" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L508" class="blob-num js-line-number" data-line-number="508"></td>
        <td id="LC508" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_HFSR_VECTTBL_Pos</span>                <span class="pl-c1">1</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB HFSR: VECTTBL Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L509" class="blob-num js-line-number" data-line-number="509"></td>
        <td id="LC509" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_HFSR_VECTTBL_Msk</span>               (<span class="pl-c1">1UL</span> &lt;&lt; SCB_HFSR_VECTTBL_Pos)                  <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB HFSR: VECTTBL Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L510" class="blob-num js-line-number" data-line-number="510"></td>
        <td id="LC510" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L511" class="blob-num js-line-number" data-line-number="511"></td>
        <td id="LC511" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> SCB Debug Fault Status Register Definitions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L512" class="blob-num js-line-number" data-line-number="512"></td>
        <td id="LC512" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_DFSR_EXTERNAL_Pos</span>               <span class="pl-c1">4</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB DFSR: EXTERNAL Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L513" class="blob-num js-line-number" data-line-number="513"></td>
        <td id="LC513" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_DFSR_EXTERNAL_Msk</span>              (<span class="pl-c1">1UL</span> &lt;&lt; SCB_DFSR_EXTERNAL_Pos)                 <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB DFSR: EXTERNAL Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L514" class="blob-num js-line-number" data-line-number="514"></td>
        <td id="LC514" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L515" class="blob-num js-line-number" data-line-number="515"></td>
        <td id="LC515" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_DFSR_VCATCH_Pos</span>                 <span class="pl-c1">3</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB DFSR: VCATCH Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L516" class="blob-num js-line-number" data-line-number="516"></td>
        <td id="LC516" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_DFSR_VCATCH_Msk</span>                (<span class="pl-c1">1UL</span> &lt;&lt; SCB_DFSR_VCATCH_Pos)                   <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB DFSR: VCATCH Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L517" class="blob-num js-line-number" data-line-number="517"></td>
        <td id="LC517" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L518" class="blob-num js-line-number" data-line-number="518"></td>
        <td id="LC518" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_DFSR_DWTTRAP_Pos</span>                <span class="pl-c1">2</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB DFSR: DWTTRAP Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L519" class="blob-num js-line-number" data-line-number="519"></td>
        <td id="LC519" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_DFSR_DWTTRAP_Msk</span>               (<span class="pl-c1">1UL</span> &lt;&lt; SCB_DFSR_DWTTRAP_Pos)                  <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB DFSR: DWTTRAP Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L520" class="blob-num js-line-number" data-line-number="520"></td>
        <td id="LC520" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L521" class="blob-num js-line-number" data-line-number="521"></td>
        <td id="LC521" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_DFSR_BKPT_Pos</span>                   <span class="pl-c1">1</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB DFSR: BKPT Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L522" class="blob-num js-line-number" data-line-number="522"></td>
        <td id="LC522" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_DFSR_BKPT_Msk</span>                  (<span class="pl-c1">1UL</span> &lt;&lt; SCB_DFSR_BKPT_Pos)                     <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB DFSR: BKPT Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L523" class="blob-num js-line-number" data-line-number="523"></td>
        <td id="LC523" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L524" class="blob-num js-line-number" data-line-number="524"></td>
        <td id="LC524" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_DFSR_HALTED_Pos</span>                 <span class="pl-c1">0</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB DFSR: HALTED Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L525" class="blob-num js-line-number" data-line-number="525"></td>
        <td id="LC525" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_DFSR_HALTED_Msk</span>                (<span class="pl-c1">1UL</span> &lt;&lt; SCB_DFSR_HALTED_Pos)                   <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB DFSR: HALTED Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L526" class="blob-num js-line-number" data-line-number="526"></td>
        <td id="LC526" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L527" class="blob-num js-line-number" data-line-number="527"></td>
        <td id="LC527" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>@} end of group CMSIS_SCB <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L528" class="blob-num js-line-number" data-line-number="528"></td>
        <td id="LC528" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L529" class="blob-num js-line-number" data-line-number="529"></td>
        <td id="LC529" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L530" class="blob-num js-line-number" data-line-number="530"></td>
        <td id="LC530" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \ingroup  CMSIS_core_register</span></td>
      </tr>
      <tr>
        <td id="L531" class="blob-num js-line-number" data-line-number="531"></td>
        <td id="LC531" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \defgroup CMSIS_SCnSCB System Controls not in SCB (SCnSCB)</span></td>
      </tr>
      <tr>
        <td id="L532" class="blob-num js-line-number" data-line-number="532"></td>
        <td id="LC532" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \brief      Type definitions for the System Control and ID Register not in the SCB</span></td>
      </tr>
      <tr>
        <td id="L533" class="blob-num js-line-number" data-line-number="533"></td>
        <td id="LC533" class="blob-code blob-code-inner js-file-line"><span class="pl-c">  @{</span></td>
      </tr>
      <tr>
        <td id="L534" class="blob-num js-line-number" data-line-number="534"></td>
        <td id="LC534" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L535" class="blob-num js-line-number" data-line-number="535"></td>
        <td id="LC535" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L536" class="blob-num js-line-number" data-line-number="536"></td>
        <td id="LC536" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \brief  Structure type to access the System Control and ID Register not in the SCB.</span></td>
      </tr>
      <tr>
        <td id="L537" class="blob-num js-line-number" data-line-number="537"></td>
        <td id="LC537" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L538" class="blob-num js-line-number" data-line-number="538"></td>
        <td id="LC538" class="blob-code blob-code-inner js-file-line"><span class="pl-k">typedef</span> <span class="pl-k">struct</span></td>
      </tr>
      <tr>
        <td id="L539" class="blob-num js-line-number" data-line-number="539"></td>
        <td id="LC539" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L540" class="blob-num js-line-number" data-line-number="540"></td>
        <td id="LC540" class="blob-code blob-code-inner js-file-line">       <span class="pl-c1">uint32_t</span> RESERVED0[<span class="pl-c1">1</span>];</td>
      </tr>
      <tr>
        <td id="L541" class="blob-num js-line-number" data-line-number="541"></td>
        <td id="LC541" class="blob-code blob-code-inner js-file-line">  __I  <span class="pl-c1">uint32_t</span> ICTR;                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x004 (R/ )  Interrupt Controller Type Register      <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L542" class="blob-num js-line-number" data-line-number="542"></td>
        <td id="LC542" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">if</span> ((defined __CM3_REV) &amp;&amp; (__CM3_REV &gt;= 0x200))</td>
      </tr>
      <tr>
        <td id="L543" class="blob-num js-line-number" data-line-number="543"></td>
        <td id="LC543" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> ACTLR;                   <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x008 (R/W)  Auxiliary Control Register      <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L544" class="blob-num js-line-number" data-line-number="544"></td>
        <td id="LC544" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">else</span></td>
      </tr>
      <tr>
        <td id="L545" class="blob-num js-line-number" data-line-number="545"></td>
        <td id="LC545" class="blob-code blob-code-inner js-file-line">       <span class="pl-c1">uint32_t</span> RESERVED1[<span class="pl-c1">1</span>];</td>
      </tr>
      <tr>
        <td id="L546" class="blob-num js-line-number" data-line-number="546"></td>
        <td id="LC546" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">endif</span></td>
      </tr>
      <tr>
        <td id="L547" class="blob-num js-line-number" data-line-number="547"></td>
        <td id="LC547" class="blob-code blob-code-inner js-file-line">} SCnSCB_Type;</td>
      </tr>
      <tr>
        <td id="L548" class="blob-num js-line-number" data-line-number="548"></td>
        <td id="LC548" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L549" class="blob-num js-line-number" data-line-number="549"></td>
        <td id="LC549" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> Interrupt Controller Type Register Definitions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L550" class="blob-num js-line-number" data-line-number="550"></td>
        <td id="LC550" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCnSCB_ICTR_INTLINESNUM_Pos</span>         <span class="pl-c1">0</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; ICTR: INTLINESNUM Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L551" class="blob-num js-line-number" data-line-number="551"></td>
        <td id="LC551" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCnSCB_ICTR_INTLINESNUM_Msk</span>        (0xFUL &lt;&lt; SCnSCB_ICTR_INTLINESNUM_Pos)      <span class="pl-c"><span class="pl-c">/*</span>!&lt; ICTR: INTLINESNUM Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L552" class="blob-num js-line-number" data-line-number="552"></td>
        <td id="LC552" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L553" class="blob-num js-line-number" data-line-number="553"></td>
        <td id="LC553" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> Auxiliary Control Register Definitions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L554" class="blob-num js-line-number" data-line-number="554"></td>
        <td id="LC554" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L555" class="blob-num js-line-number" data-line-number="555"></td>
        <td id="LC555" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCnSCB_ACTLR_DISFOLD_Pos</span>            <span class="pl-c1">2</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; ACTLR: DISFOLD Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L556" class="blob-num js-line-number" data-line-number="556"></td>
        <td id="LC556" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCnSCB_ACTLR_DISFOLD_Msk</span>           (<span class="pl-c1">1UL</span> &lt;&lt; SCnSCB_ACTLR_DISFOLD_Pos)           <span class="pl-c"><span class="pl-c">/*</span>!&lt; ACTLR: DISFOLD Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L557" class="blob-num js-line-number" data-line-number="557"></td>
        <td id="LC557" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L558" class="blob-num js-line-number" data-line-number="558"></td>
        <td id="LC558" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCnSCB_ACTLR_DISDEFWBUF_Pos</span>         <span class="pl-c1">1</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; ACTLR: DISDEFWBUF Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L559" class="blob-num js-line-number" data-line-number="559"></td>
        <td id="LC559" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCnSCB_ACTLR_DISDEFWBUF_Msk</span>        (<span class="pl-c1">1UL</span> &lt;&lt; SCnSCB_ACTLR_DISDEFWBUF_Pos)        <span class="pl-c"><span class="pl-c">/*</span>!&lt; ACTLR: DISDEFWBUF Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L560" class="blob-num js-line-number" data-line-number="560"></td>
        <td id="LC560" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L561" class="blob-num js-line-number" data-line-number="561"></td>
        <td id="LC561" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCnSCB_ACTLR_DISMCYCINT_Pos</span>         <span class="pl-c1">0</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; ACTLR: DISMCYCINT Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L562" class="blob-num js-line-number" data-line-number="562"></td>
        <td id="LC562" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCnSCB_ACTLR_DISMCYCINT_Msk</span>        (<span class="pl-c1">1UL</span> &lt;&lt; SCnSCB_ACTLR_DISMCYCINT_Pos)        <span class="pl-c"><span class="pl-c">/*</span>!&lt; ACTLR: DISMCYCINT Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L563" class="blob-num js-line-number" data-line-number="563"></td>
        <td id="LC563" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L564" class="blob-num js-line-number" data-line-number="564"></td>
        <td id="LC564" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>@} end of group CMSIS_SCnotSCB <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L565" class="blob-num js-line-number" data-line-number="565"></td>
        <td id="LC565" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L566" class="blob-num js-line-number" data-line-number="566"></td>
        <td id="LC566" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L567" class="blob-num js-line-number" data-line-number="567"></td>
        <td id="LC567" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \ingroup  CMSIS_core_register</span></td>
      </tr>
      <tr>
        <td id="L568" class="blob-num js-line-number" data-line-number="568"></td>
        <td id="LC568" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \defgroup CMSIS_SysTick     System Tick Timer (SysTick)</span></td>
      </tr>
      <tr>
        <td id="L569" class="blob-num js-line-number" data-line-number="569"></td>
        <td id="LC569" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \brief      Type definitions for the System Timer Registers.</span></td>
      </tr>
      <tr>
        <td id="L570" class="blob-num js-line-number" data-line-number="570"></td>
        <td id="LC570" class="blob-code blob-code-inner js-file-line"><span class="pl-c">  @{</span></td>
      </tr>
      <tr>
        <td id="L571" class="blob-num js-line-number" data-line-number="571"></td>
        <td id="LC571" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L572" class="blob-num js-line-number" data-line-number="572"></td>
        <td id="LC572" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L573" class="blob-num js-line-number" data-line-number="573"></td>
        <td id="LC573" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \brief  Structure type to access the System Timer (SysTick).</span></td>
      </tr>
      <tr>
        <td id="L574" class="blob-num js-line-number" data-line-number="574"></td>
        <td id="LC574" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L575" class="blob-num js-line-number" data-line-number="575"></td>
        <td id="LC575" class="blob-code blob-code-inner js-file-line"><span class="pl-k">typedef</span> <span class="pl-k">struct</span></td>
      </tr>
      <tr>
        <td id="L576" class="blob-num js-line-number" data-line-number="576"></td>
        <td id="LC576" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L577" class="blob-num js-line-number" data-line-number="577"></td>
        <td id="LC577" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> CTRL;                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x000 (R/W)  SysTick Control and Status Register <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L578" class="blob-num js-line-number" data-line-number="578"></td>
        <td id="LC578" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> LOAD;                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x004 (R/W)  SysTick Reload Value Register       <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L579" class="blob-num js-line-number" data-line-number="579"></td>
        <td id="LC579" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> VAL;                     <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x008 (R/W)  SysTick Current Value Register      <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L580" class="blob-num js-line-number" data-line-number="580"></td>
        <td id="LC580" class="blob-code blob-code-inner js-file-line">  __I  <span class="pl-c1">uint32_t</span> CALIB;                   <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x00C (R/ )  SysTick Calibration Register        <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L581" class="blob-num js-line-number" data-line-number="581"></td>
        <td id="LC581" class="blob-code blob-code-inner js-file-line">} SysTick_Type;</td>
      </tr>
      <tr>
        <td id="L582" class="blob-num js-line-number" data-line-number="582"></td>
        <td id="LC582" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L583" class="blob-num js-line-number" data-line-number="583"></td>
        <td id="LC583" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> SysTick Control / Status Register Definitions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L584" class="blob-num js-line-number" data-line-number="584"></td>
        <td id="LC584" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SysTick_CTRL_COUNTFLAG_Pos</span>         <span class="pl-c1">16</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SysTick CTRL: COUNTFLAG Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L585" class="blob-num js-line-number" data-line-number="585"></td>
        <td id="LC585" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SysTick_CTRL_COUNTFLAG_Msk</span>         (<span class="pl-c1">1UL</span> &lt;&lt; SysTick_CTRL_COUNTFLAG_Pos)            <span class="pl-c"><span class="pl-c">/*</span>!&lt; SysTick CTRL: COUNTFLAG Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L586" class="blob-num js-line-number" data-line-number="586"></td>
        <td id="LC586" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L587" class="blob-num js-line-number" data-line-number="587"></td>
        <td id="LC587" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SysTick_CTRL_CLKSOURCE_Pos</span>          <span class="pl-c1">2</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SysTick CTRL: CLKSOURCE Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L588" class="blob-num js-line-number" data-line-number="588"></td>
        <td id="LC588" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SysTick_CTRL_CLKSOURCE_Msk</span>         (<span class="pl-c1">1UL</span> &lt;&lt; SysTick_CTRL_CLKSOURCE_Pos)            <span class="pl-c"><span class="pl-c">/*</span>!&lt; SysTick CTRL: CLKSOURCE Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L589" class="blob-num js-line-number" data-line-number="589"></td>
        <td id="LC589" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L590" class="blob-num js-line-number" data-line-number="590"></td>
        <td id="LC590" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SysTick_CTRL_TICKINT_Pos</span>            <span class="pl-c1">1</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SysTick CTRL: TICKINT Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L591" class="blob-num js-line-number" data-line-number="591"></td>
        <td id="LC591" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SysTick_CTRL_TICKINT_Msk</span>           (<span class="pl-c1">1UL</span> &lt;&lt; SysTick_CTRL_TICKINT_Pos)              <span class="pl-c"><span class="pl-c">/*</span>!&lt; SysTick CTRL: TICKINT Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L592" class="blob-num js-line-number" data-line-number="592"></td>
        <td id="LC592" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L593" class="blob-num js-line-number" data-line-number="593"></td>
        <td id="LC593" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SysTick_CTRL_ENABLE_Pos</span>             <span class="pl-c1">0</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SysTick CTRL: ENABLE Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L594" class="blob-num js-line-number" data-line-number="594"></td>
        <td id="LC594" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SysTick_CTRL_ENABLE_Msk</span>            (<span class="pl-c1">1UL</span> &lt;&lt; SysTick_CTRL_ENABLE_Pos)               <span class="pl-c"><span class="pl-c">/*</span>!&lt; SysTick CTRL: ENABLE Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L595" class="blob-num js-line-number" data-line-number="595"></td>
        <td id="LC595" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L596" class="blob-num js-line-number" data-line-number="596"></td>
        <td id="LC596" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> SysTick Reload Register Definitions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L597" class="blob-num js-line-number" data-line-number="597"></td>
        <td id="LC597" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SysTick_LOAD_RELOAD_Pos</span>             <span class="pl-c1">0</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SysTick LOAD: RELOAD Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L598" class="blob-num js-line-number" data-line-number="598"></td>
        <td id="LC598" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SysTick_LOAD_RELOAD_Msk</span>            (0xFFFFFFUL &lt;&lt; SysTick_LOAD_RELOAD_Pos)        <span class="pl-c"><span class="pl-c">/*</span>!&lt; SysTick LOAD: RELOAD Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L599" class="blob-num js-line-number" data-line-number="599"></td>
        <td id="LC599" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L600" class="blob-num js-line-number" data-line-number="600"></td>
        <td id="LC600" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> SysTick Current Register Definitions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L601" class="blob-num js-line-number" data-line-number="601"></td>
        <td id="LC601" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SysTick_VAL_CURRENT_Pos</span>             <span class="pl-c1">0</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SysTick VAL: CURRENT Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L602" class="blob-num js-line-number" data-line-number="602"></td>
        <td id="LC602" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SysTick_VAL_CURRENT_Msk</span>            (0xFFFFFFUL &lt;&lt; SysTick_VAL_CURRENT_Pos)        <span class="pl-c"><span class="pl-c">/*</span>!&lt; SysTick VAL: CURRENT Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L603" class="blob-num js-line-number" data-line-number="603"></td>
        <td id="LC603" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L604" class="blob-num js-line-number" data-line-number="604"></td>
        <td id="LC604" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> SysTick Calibration Register Definitions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L605" class="blob-num js-line-number" data-line-number="605"></td>
        <td id="LC605" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SysTick_CALIB_NOREF_Pos</span>            <span class="pl-c1">31</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SysTick CALIB: NOREF Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L606" class="blob-num js-line-number" data-line-number="606"></td>
        <td id="LC606" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SysTick_CALIB_NOREF_Msk</span>            (<span class="pl-c1">1UL</span> &lt;&lt; SysTick_CALIB_NOREF_Pos)               <span class="pl-c"><span class="pl-c">/*</span>!&lt; SysTick CALIB: NOREF Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L607" class="blob-num js-line-number" data-line-number="607"></td>
        <td id="LC607" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L608" class="blob-num js-line-number" data-line-number="608"></td>
        <td id="LC608" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SysTick_CALIB_SKEW_Pos</span>             <span class="pl-c1">30</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SysTick CALIB: SKEW Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L609" class="blob-num js-line-number" data-line-number="609"></td>
        <td id="LC609" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SysTick_CALIB_SKEW_Msk</span>             (<span class="pl-c1">1UL</span> &lt;&lt; SysTick_CALIB_SKEW_Pos)                <span class="pl-c"><span class="pl-c">/*</span>!&lt; SysTick CALIB: SKEW Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L610" class="blob-num js-line-number" data-line-number="610"></td>
        <td id="LC610" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L611" class="blob-num js-line-number" data-line-number="611"></td>
        <td id="LC611" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SysTick_CALIB_TENMS_Pos</span>             <span class="pl-c1">0</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; SysTick CALIB: TENMS Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L612" class="blob-num js-line-number" data-line-number="612"></td>
        <td id="LC612" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SysTick_CALIB_TENMS_Msk</span>            (0xFFFFFFUL &lt;&lt; SysTick_VAL_CURRENT_Pos)        <span class="pl-c"><span class="pl-c">/*</span>!&lt; SysTick CALIB: TENMS Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L613" class="blob-num js-line-number" data-line-number="613"></td>
        <td id="LC613" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L614" class="blob-num js-line-number" data-line-number="614"></td>
        <td id="LC614" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>@} end of group CMSIS_SysTick <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L615" class="blob-num js-line-number" data-line-number="615"></td>
        <td id="LC615" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L616" class="blob-num js-line-number" data-line-number="616"></td>
        <td id="LC616" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L617" class="blob-num js-line-number" data-line-number="617"></td>
        <td id="LC617" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \ingroup  CMSIS_core_register</span></td>
      </tr>
      <tr>
        <td id="L618" class="blob-num js-line-number" data-line-number="618"></td>
        <td id="LC618" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \defgroup CMSIS_ITM     Instrumentation Trace Macrocell (ITM)</span></td>
      </tr>
      <tr>
        <td id="L619" class="blob-num js-line-number" data-line-number="619"></td>
        <td id="LC619" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \brief      Type definitions for the Instrumentation Trace Macrocell (ITM)</span></td>
      </tr>
      <tr>
        <td id="L620" class="blob-num js-line-number" data-line-number="620"></td>
        <td id="LC620" class="blob-code blob-code-inner js-file-line"><span class="pl-c">  @{</span></td>
      </tr>
      <tr>
        <td id="L621" class="blob-num js-line-number" data-line-number="621"></td>
        <td id="LC621" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L622" class="blob-num js-line-number" data-line-number="622"></td>
        <td id="LC622" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L623" class="blob-num js-line-number" data-line-number="623"></td>
        <td id="LC623" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \brief  Structure type to access the Instrumentation Trace Macrocell Register (ITM).</span></td>
      </tr>
      <tr>
        <td id="L624" class="blob-num js-line-number" data-line-number="624"></td>
        <td id="LC624" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L625" class="blob-num js-line-number" data-line-number="625"></td>
        <td id="LC625" class="blob-code blob-code-inner js-file-line"><span class="pl-k">typedef</span> <span class="pl-k">struct</span></td>
      </tr>
      <tr>
        <td id="L626" class="blob-num js-line-number" data-line-number="626"></td>
        <td id="LC626" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L627" class="blob-num js-line-number" data-line-number="627"></td>
        <td id="LC627" class="blob-code blob-code-inner js-file-line">  __O  <span class="pl-k">union</span></td>
      </tr>
      <tr>
        <td id="L628" class="blob-num js-line-number" data-line-number="628"></td>
        <td id="LC628" class="blob-code blob-code-inner js-file-line">  {</td>
      </tr>
      <tr>
        <td id="L629" class="blob-num js-line-number" data-line-number="629"></td>
        <td id="LC629" class="blob-code blob-code-inner js-file-line">    __O  <span class="pl-c1">uint8_t</span>    u8;                  <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x000 ( /W)  ITM Stimulus Port 8-bit                   <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L630" class="blob-num js-line-number" data-line-number="630"></td>
        <td id="LC630" class="blob-code blob-code-inner js-file-line">    __O  <span class="pl-c1">uint16_t</span>   u16;                 <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x000 ( /W)  ITM Stimulus Port 16-bit                  <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L631" class="blob-num js-line-number" data-line-number="631"></td>
        <td id="LC631" class="blob-code blob-code-inner js-file-line">    __O  <span class="pl-c1">uint32_t</span>   u32;                 <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x000 ( /W)  ITM Stimulus Port 32-bit                  <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L632" class="blob-num js-line-number" data-line-number="632"></td>
        <td id="LC632" class="blob-code blob-code-inner js-file-line">  }  PORT [<span class="pl-c1">32</span>];                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x000 ( /W)  ITM Stimulus Port Registers               <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L633" class="blob-num js-line-number" data-line-number="633"></td>
        <td id="LC633" class="blob-code blob-code-inner js-file-line">       <span class="pl-c1">uint32_t</span> RESERVED0[<span class="pl-c1">864</span>];</td>
      </tr>
      <tr>
        <td id="L634" class="blob-num js-line-number" data-line-number="634"></td>
        <td id="LC634" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> TER;                     <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0xE00 (R/W)  ITM Trace Enable Register                 <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L635" class="blob-num js-line-number" data-line-number="635"></td>
        <td id="LC635" class="blob-code blob-code-inner js-file-line">       <span class="pl-c1">uint32_t</span> RESERVED1[<span class="pl-c1">15</span>];</td>
      </tr>
      <tr>
        <td id="L636" class="blob-num js-line-number" data-line-number="636"></td>
        <td id="LC636" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> TPR;                     <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0xE40 (R/W)  ITM Trace Privilege Register              <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L637" class="blob-num js-line-number" data-line-number="637"></td>
        <td id="LC637" class="blob-code blob-code-inner js-file-line">       <span class="pl-c1">uint32_t</span> RESERVED2[<span class="pl-c1">15</span>];</td>
      </tr>
      <tr>
        <td id="L638" class="blob-num js-line-number" data-line-number="638"></td>
        <td id="LC638" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> TCR;                     <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0xE80 (R/W)  ITM Trace Control Register                <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L639" class="blob-num js-line-number" data-line-number="639"></td>
        <td id="LC639" class="blob-code blob-code-inner js-file-line">       <span class="pl-c1">uint32_t</span> RESERVED3[<span class="pl-c1">29</span>];                                  </td>
      </tr>
      <tr>
        <td id="L640" class="blob-num js-line-number" data-line-number="640"></td>
        <td id="LC640" class="blob-code blob-code-inner js-file-line">  __O  <span class="pl-c1">uint32_t</span> IWR;                     <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0xEF8 ( /W)  ITM Integration Write Register            <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L641" class="blob-num js-line-number" data-line-number="641"></td>
        <td id="LC641" class="blob-code blob-code-inner js-file-line">  __I  <span class="pl-c1">uint32_t</span> IRR;                     <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0xEFC (R/ )  ITM Integration Read Register             <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L642" class="blob-num js-line-number" data-line-number="642"></td>
        <td id="LC642" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> IMCR;                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0xF00 (R/W)  ITM Integration Mode Control Register     <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L643" class="blob-num js-line-number" data-line-number="643"></td>
        <td id="LC643" class="blob-code blob-code-inner js-file-line">       <span class="pl-c1">uint32_t</span> RESERVED4[<span class="pl-c1">43</span>];                                  </td>
      </tr>
      <tr>
        <td id="L644" class="blob-num js-line-number" data-line-number="644"></td>
        <td id="LC644" class="blob-code blob-code-inner js-file-line">  __O  <span class="pl-c1">uint32_t</span> LAR;                     <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0xFB0 ( /W)  ITM Lock Access Register                  <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L645" class="blob-num js-line-number" data-line-number="645"></td>
        <td id="LC645" class="blob-code blob-code-inner js-file-line">  __I  <span class="pl-c1">uint32_t</span> LSR;                     <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0xFB4 (R/ )  ITM Lock Status Register                  <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L646" class="blob-num js-line-number" data-line-number="646"></td>
        <td id="LC646" class="blob-code blob-code-inner js-file-line">       <span class="pl-c1">uint32_t</span> RESERVED5[<span class="pl-c1">6</span>];                                   </td>
      </tr>
      <tr>
        <td id="L647" class="blob-num js-line-number" data-line-number="647"></td>
        <td id="LC647" class="blob-code blob-code-inner js-file-line">  __I  <span class="pl-c1">uint32_t</span> PID4;                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0xFD0 (R/ )  ITM Peripheral Identification Register #4 <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L648" class="blob-num js-line-number" data-line-number="648"></td>
        <td id="LC648" class="blob-code blob-code-inner js-file-line">  __I  <span class="pl-c1">uint32_t</span> PID5;                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0xFD4 (R/ )  ITM Peripheral Identification Register #5 <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L649" class="blob-num js-line-number" data-line-number="649"></td>
        <td id="LC649" class="blob-code blob-code-inner js-file-line">  __I  <span class="pl-c1">uint32_t</span> PID6;                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0xFD8 (R/ )  ITM Peripheral Identification Register #6 <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L650" class="blob-num js-line-number" data-line-number="650"></td>
        <td id="LC650" class="blob-code blob-code-inner js-file-line">  __I  <span class="pl-c1">uint32_t</span> PID7;                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0xFDC (R/ )  ITM Peripheral Identification Register #7 <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L651" class="blob-num js-line-number" data-line-number="651"></td>
        <td id="LC651" class="blob-code blob-code-inner js-file-line">  __I  <span class="pl-c1">uint32_t</span> PID0;                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0xFE0 (R/ )  ITM Peripheral Identification Register #0 <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L652" class="blob-num js-line-number" data-line-number="652"></td>
        <td id="LC652" class="blob-code blob-code-inner js-file-line">  __I  <span class="pl-c1">uint32_t</span> PID1;                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0xFE4 (R/ )  ITM Peripheral Identification Register #1 <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L653" class="blob-num js-line-number" data-line-number="653"></td>
        <td id="LC653" class="blob-code blob-code-inner js-file-line">  __I  <span class="pl-c1">uint32_t</span> PID2;                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0xFE8 (R/ )  ITM Peripheral Identification Register #2 <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L654" class="blob-num js-line-number" data-line-number="654"></td>
        <td id="LC654" class="blob-code blob-code-inner js-file-line">  __I  <span class="pl-c1">uint32_t</span> PID3;                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0xFEC (R/ )  ITM Peripheral Identification Register #3 <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L655" class="blob-num js-line-number" data-line-number="655"></td>
        <td id="LC655" class="blob-code blob-code-inner js-file-line">  __I  <span class="pl-c1">uint32_t</span> CID0;                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0xFF0 (R/ )  ITM Component  Identification Register #0 <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L656" class="blob-num js-line-number" data-line-number="656"></td>
        <td id="LC656" class="blob-code blob-code-inner js-file-line">  __I  <span class="pl-c1">uint32_t</span> CID1;                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0xFF4 (R/ )  ITM Component  Identification Register #1 <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L657" class="blob-num js-line-number" data-line-number="657"></td>
        <td id="LC657" class="blob-code blob-code-inner js-file-line">  __I  <span class="pl-c1">uint32_t</span> CID2;                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0xFF8 (R/ )  ITM Component  Identification Register #2 <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L658" class="blob-num js-line-number" data-line-number="658"></td>
        <td id="LC658" class="blob-code blob-code-inner js-file-line">  __I  <span class="pl-c1">uint32_t</span> CID3;                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0xFFC (R/ )  ITM Component  Identification Register #3 <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L659" class="blob-num js-line-number" data-line-number="659"></td>
        <td id="LC659" class="blob-code blob-code-inner js-file-line">} ITM_Type;</td>
      </tr>
      <tr>
        <td id="L660" class="blob-num js-line-number" data-line-number="660"></td>
        <td id="LC660" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L661" class="blob-num js-line-number" data-line-number="661"></td>
        <td id="LC661" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> ITM Trace Privilege Register Definitions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L662" class="blob-num js-line-number" data-line-number="662"></td>
        <td id="LC662" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">ITM_TPR_PRIVMASK_Pos</span>                <span class="pl-c1">0</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; ITM TPR: PRIVMASK Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L663" class="blob-num js-line-number" data-line-number="663"></td>
        <td id="LC663" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">ITM_TPR_PRIVMASK_Msk</span>               (0xFUL &lt;&lt; ITM_TPR_PRIVMASK_Pos)                <span class="pl-c"><span class="pl-c">/*</span>!&lt; ITM TPR: PRIVMASK Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L664" class="blob-num js-line-number" data-line-number="664"></td>
        <td id="LC664" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L665" class="blob-num js-line-number" data-line-number="665"></td>
        <td id="LC665" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> ITM Trace Control Register Definitions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L666" class="blob-num js-line-number" data-line-number="666"></td>
        <td id="LC666" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">ITM_TCR_BUSY_Pos</span>                   <span class="pl-c1">23</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; ITM TCR: BUSY Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L667" class="blob-num js-line-number" data-line-number="667"></td>
        <td id="LC667" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">ITM_TCR_BUSY_Msk</span>                   (<span class="pl-c1">1UL</span> &lt;&lt; ITM_TCR_BUSY_Pos)                      <span class="pl-c"><span class="pl-c">/*</span>!&lt; ITM TCR: BUSY Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L668" class="blob-num js-line-number" data-line-number="668"></td>
        <td id="LC668" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L669" class="blob-num js-line-number" data-line-number="669"></td>
        <td id="LC669" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">ITM_TCR_TraceBusID_Pos</span>             <span class="pl-c1">16</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; ITM TCR: ATBID Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L670" class="blob-num js-line-number" data-line-number="670"></td>
        <td id="LC670" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">ITM_TCR_TraceBusID_Msk</span>             (0x7FUL &lt;&lt; ITM_TCR_TraceBusID_Pos)             <span class="pl-c"><span class="pl-c">/*</span>!&lt; ITM TCR: ATBID Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L671" class="blob-num js-line-number" data-line-number="671"></td>
        <td id="LC671" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L672" class="blob-num js-line-number" data-line-number="672"></td>
        <td id="LC672" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">ITM_TCR_GTSFREQ_Pos</span>                <span class="pl-c1">10</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; ITM TCR: Global timestamp frequency Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L673" class="blob-num js-line-number" data-line-number="673"></td>
        <td id="LC673" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">ITM_TCR_GTSFREQ_Msk</span>                (<span class="pl-c1">3UL</span> &lt;&lt; ITM_TCR_GTSFREQ_Pos)                   <span class="pl-c"><span class="pl-c">/*</span>!&lt; ITM TCR: Global timestamp frequency Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L674" class="blob-num js-line-number" data-line-number="674"></td>
        <td id="LC674" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L675" class="blob-num js-line-number" data-line-number="675"></td>
        <td id="LC675" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">ITM_TCR_TSPrescale_Pos</span>              <span class="pl-c1">8</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; ITM TCR: TSPrescale Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L676" class="blob-num js-line-number" data-line-number="676"></td>
        <td id="LC676" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">ITM_TCR_TSPrescale_Msk</span>             (<span class="pl-c1">3UL</span> &lt;&lt; ITM_TCR_TSPrescale_Pos)                <span class="pl-c"><span class="pl-c">/*</span>!&lt; ITM TCR: TSPrescale Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L677" class="blob-num js-line-number" data-line-number="677"></td>
        <td id="LC677" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L678" class="blob-num js-line-number" data-line-number="678"></td>
        <td id="LC678" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">ITM_TCR_SWOENA_Pos</span>                  <span class="pl-c1">4</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; ITM TCR: SWOENA Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L679" class="blob-num js-line-number" data-line-number="679"></td>
        <td id="LC679" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">ITM_TCR_SWOENA_Msk</span>                 (<span class="pl-c1">1UL</span> &lt;&lt; ITM_TCR_SWOENA_Pos)                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; ITM TCR: SWOENA Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L680" class="blob-num js-line-number" data-line-number="680"></td>
        <td id="LC680" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L681" class="blob-num js-line-number" data-line-number="681"></td>
        <td id="LC681" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">ITM_TCR_DWTENA_Pos</span>                  <span class="pl-c1">3</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; ITM TCR: DWTENA Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L682" class="blob-num js-line-number" data-line-number="682"></td>
        <td id="LC682" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">ITM_TCR_DWTENA_Msk</span>                 (<span class="pl-c1">1UL</span> &lt;&lt; ITM_TCR_DWTENA_Pos)                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; ITM TCR: DWTENA Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L683" class="blob-num js-line-number" data-line-number="683"></td>
        <td id="LC683" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L684" class="blob-num js-line-number" data-line-number="684"></td>
        <td id="LC684" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">ITM_TCR_SYNCENA_Pos</span>                 <span class="pl-c1">2</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; ITM TCR: SYNCENA Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L685" class="blob-num js-line-number" data-line-number="685"></td>
        <td id="LC685" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">ITM_TCR_SYNCENA_Msk</span>                (<span class="pl-c1">1UL</span> &lt;&lt; ITM_TCR_SYNCENA_Pos)                   <span class="pl-c"><span class="pl-c">/*</span>!&lt; ITM TCR: SYNCENA Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L686" class="blob-num js-line-number" data-line-number="686"></td>
        <td id="LC686" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L687" class="blob-num js-line-number" data-line-number="687"></td>
        <td id="LC687" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">ITM_TCR_TSENA_Pos</span>                   <span class="pl-c1">1</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; ITM TCR: TSENA Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L688" class="blob-num js-line-number" data-line-number="688"></td>
        <td id="LC688" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">ITM_TCR_TSENA_Msk</span>                  (<span class="pl-c1">1UL</span> &lt;&lt; ITM_TCR_TSENA_Pos)                     <span class="pl-c"><span class="pl-c">/*</span>!&lt; ITM TCR: TSENA Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L689" class="blob-num js-line-number" data-line-number="689"></td>
        <td id="LC689" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L690" class="blob-num js-line-number" data-line-number="690"></td>
        <td id="LC690" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">ITM_TCR_ITMENA_Pos</span>                  <span class="pl-c1">0</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; ITM TCR: ITM Enable bit Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L691" class="blob-num js-line-number" data-line-number="691"></td>
        <td id="LC691" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">ITM_TCR_ITMENA_Msk</span>                 (<span class="pl-c1">1UL</span> &lt;&lt; ITM_TCR_ITMENA_Pos)                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; ITM TCR: ITM Enable bit Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L692" class="blob-num js-line-number" data-line-number="692"></td>
        <td id="LC692" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L693" class="blob-num js-line-number" data-line-number="693"></td>
        <td id="LC693" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> ITM Integration Write Register Definitions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L694" class="blob-num js-line-number" data-line-number="694"></td>
        <td id="LC694" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">ITM_IWR_ATVALIDM_Pos</span>                <span class="pl-c1">0</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; ITM IWR: ATVALIDM Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L695" class="blob-num js-line-number" data-line-number="695"></td>
        <td id="LC695" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">ITM_IWR_ATVALIDM_Msk</span>               (<span class="pl-c1">1UL</span> &lt;&lt; ITM_IWR_ATVALIDM_Pos)                  <span class="pl-c"><span class="pl-c">/*</span>!&lt; ITM IWR: ATVALIDM Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L696" class="blob-num js-line-number" data-line-number="696"></td>
        <td id="LC696" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L697" class="blob-num js-line-number" data-line-number="697"></td>
        <td id="LC697" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> ITM Integration Read Register Definitions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L698" class="blob-num js-line-number" data-line-number="698"></td>
        <td id="LC698" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">ITM_IRR_ATREADYM_Pos</span>                <span class="pl-c1">0</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; ITM IRR: ATREADYM Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L699" class="blob-num js-line-number" data-line-number="699"></td>
        <td id="LC699" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">ITM_IRR_ATREADYM_Msk</span>               (<span class="pl-c1">1UL</span> &lt;&lt; ITM_IRR_ATREADYM_Pos)                  <span class="pl-c"><span class="pl-c">/*</span>!&lt; ITM IRR: ATREADYM Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L700" class="blob-num js-line-number" data-line-number="700"></td>
        <td id="LC700" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L701" class="blob-num js-line-number" data-line-number="701"></td>
        <td id="LC701" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> ITM Integration Mode Control Register Definitions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L702" class="blob-num js-line-number" data-line-number="702"></td>
        <td id="LC702" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">ITM_IMCR_INTEGRATION_Pos</span>            <span class="pl-c1">0</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; ITM IMCR: INTEGRATION Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L703" class="blob-num js-line-number" data-line-number="703"></td>
        <td id="LC703" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">ITM_IMCR_INTEGRATION_Msk</span>           (<span class="pl-c1">1UL</span> &lt;&lt; ITM_IMCR_INTEGRATION_Pos)              <span class="pl-c"><span class="pl-c">/*</span>!&lt; ITM IMCR: INTEGRATION Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L704" class="blob-num js-line-number" data-line-number="704"></td>
        <td id="LC704" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L705" class="blob-num js-line-number" data-line-number="705"></td>
        <td id="LC705" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> ITM Lock Status Register Definitions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L706" class="blob-num js-line-number" data-line-number="706"></td>
        <td id="LC706" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">ITM_LSR_ByteAcc_Pos</span>                 <span class="pl-c1">2</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; ITM LSR: ByteAcc Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L707" class="blob-num js-line-number" data-line-number="707"></td>
        <td id="LC707" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">ITM_LSR_ByteAcc_Msk</span>                (<span class="pl-c1">1UL</span> &lt;&lt; ITM_LSR_ByteAcc_Pos)                   <span class="pl-c"><span class="pl-c">/*</span>!&lt; ITM LSR: ByteAcc Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L708" class="blob-num js-line-number" data-line-number="708"></td>
        <td id="LC708" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L709" class="blob-num js-line-number" data-line-number="709"></td>
        <td id="LC709" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">ITM_LSR_Access_Pos</span>                  <span class="pl-c1">1</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; ITM LSR: Access Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L710" class="blob-num js-line-number" data-line-number="710"></td>
        <td id="LC710" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">ITM_LSR_Access_Msk</span>                 (<span class="pl-c1">1UL</span> &lt;&lt; ITM_LSR_Access_Pos)                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; ITM LSR: Access Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L711" class="blob-num js-line-number" data-line-number="711"></td>
        <td id="LC711" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L712" class="blob-num js-line-number" data-line-number="712"></td>
        <td id="LC712" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">ITM_LSR_Present_Pos</span>                 <span class="pl-c1">0</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; ITM LSR: Present Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L713" class="blob-num js-line-number" data-line-number="713"></td>
        <td id="LC713" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">ITM_LSR_Present_Msk</span>                (<span class="pl-c1">1UL</span> &lt;&lt; ITM_LSR_Present_Pos)                   <span class="pl-c"><span class="pl-c">/*</span>!&lt; ITM LSR: Present Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L714" class="blob-num js-line-number" data-line-number="714"></td>
        <td id="LC714" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L715" class="blob-num js-line-number" data-line-number="715"></td>
        <td id="LC715" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>@}<span class="pl-c">*/</span></span> <span class="pl-c"><span class="pl-c">/*</span> end of group CMSIS_ITM <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L716" class="blob-num js-line-number" data-line-number="716"></td>
        <td id="LC716" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L717" class="blob-num js-line-number" data-line-number="717"></td>
        <td id="LC717" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L718" class="blob-num js-line-number" data-line-number="718"></td>
        <td id="LC718" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \ingroup  CMSIS_core_register</span></td>
      </tr>
      <tr>
        <td id="L719" class="blob-num js-line-number" data-line-number="719"></td>
        <td id="LC719" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \defgroup CMSIS_DWT     Data Watchpoint and Trace (DWT)</span></td>
      </tr>
      <tr>
        <td id="L720" class="blob-num js-line-number" data-line-number="720"></td>
        <td id="LC720" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \brief      Type definitions for the Data Watchpoint and Trace (DWT)</span></td>
      </tr>
      <tr>
        <td id="L721" class="blob-num js-line-number" data-line-number="721"></td>
        <td id="LC721" class="blob-code blob-code-inner js-file-line"><span class="pl-c">  @{</span></td>
      </tr>
      <tr>
        <td id="L722" class="blob-num js-line-number" data-line-number="722"></td>
        <td id="LC722" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L723" class="blob-num js-line-number" data-line-number="723"></td>
        <td id="LC723" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L724" class="blob-num js-line-number" data-line-number="724"></td>
        <td id="LC724" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \brief  Structure type to access the Data Watchpoint and Trace Register (DWT).</span></td>
      </tr>
      <tr>
        <td id="L725" class="blob-num js-line-number" data-line-number="725"></td>
        <td id="LC725" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L726" class="blob-num js-line-number" data-line-number="726"></td>
        <td id="LC726" class="blob-code blob-code-inner js-file-line"><span class="pl-k">typedef</span> <span class="pl-k">struct</span></td>
      </tr>
      <tr>
        <td id="L727" class="blob-num js-line-number" data-line-number="727"></td>
        <td id="LC727" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L728" class="blob-num js-line-number" data-line-number="728"></td>
        <td id="LC728" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> CTRL;                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x000 (R/W)  Control Register                          <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L729" class="blob-num js-line-number" data-line-number="729"></td>
        <td id="LC729" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> CYCCNT;                  <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x004 (R/W)  Cycle Count Register                      <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L730" class="blob-num js-line-number" data-line-number="730"></td>
        <td id="LC730" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> CPICNT;                  <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x008 (R/W)  CPI Count Register                        <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L731" class="blob-num js-line-number" data-line-number="731"></td>
        <td id="LC731" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> EXCCNT;                  <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x00C (R/W)  Exception Overhead Count Register         <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L732" class="blob-num js-line-number" data-line-number="732"></td>
        <td id="LC732" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> SLEEPCNT;                <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x010 (R/W)  Sleep Count Register                      <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L733" class="blob-num js-line-number" data-line-number="733"></td>
        <td id="LC733" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> LSUCNT;                  <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x014 (R/W)  LSU Count Register                        <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L734" class="blob-num js-line-number" data-line-number="734"></td>
        <td id="LC734" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> FOLDCNT;                 <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x018 (R/W)  Folded-instruction Count Register         <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L735" class="blob-num js-line-number" data-line-number="735"></td>
        <td id="LC735" class="blob-code blob-code-inner js-file-line">  __I  <span class="pl-c1">uint32_t</span> PCSR;                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x01C (R/ )  Program Counter Sample Register           <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L736" class="blob-num js-line-number" data-line-number="736"></td>
        <td id="LC736" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> COMP0;                   <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x020 (R/W)  Comparator Register 0                     <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L737" class="blob-num js-line-number" data-line-number="737"></td>
        <td id="LC737" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> MASK0;                   <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x024 (R/W)  Mask Register 0                           <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L738" class="blob-num js-line-number" data-line-number="738"></td>
        <td id="LC738" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> FUNCTION0;               <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x028 (R/W)  Function Register 0                       <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L739" class="blob-num js-line-number" data-line-number="739"></td>
        <td id="LC739" class="blob-code blob-code-inner js-file-line">       <span class="pl-c1">uint32_t</span> RESERVED0[<span class="pl-c1">1</span>];</td>
      </tr>
      <tr>
        <td id="L740" class="blob-num js-line-number" data-line-number="740"></td>
        <td id="LC740" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> COMP1;                   <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x030 (R/W)  Comparator Register 1                     <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L741" class="blob-num js-line-number" data-line-number="741"></td>
        <td id="LC741" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> MASK1;                   <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x034 (R/W)  Mask Register 1                           <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L742" class="blob-num js-line-number" data-line-number="742"></td>
        <td id="LC742" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> FUNCTION1;               <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x038 (R/W)  Function Register 1                       <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L743" class="blob-num js-line-number" data-line-number="743"></td>
        <td id="LC743" class="blob-code blob-code-inner js-file-line">       <span class="pl-c1">uint32_t</span> RESERVED1[<span class="pl-c1">1</span>];</td>
      </tr>
      <tr>
        <td id="L744" class="blob-num js-line-number" data-line-number="744"></td>
        <td id="LC744" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> COMP2;                   <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x040 (R/W)  Comparator Register 2                     <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L745" class="blob-num js-line-number" data-line-number="745"></td>
        <td id="LC745" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> MASK2;                   <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x044 (R/W)  Mask Register 2                           <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L746" class="blob-num js-line-number" data-line-number="746"></td>
        <td id="LC746" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> FUNCTION2;               <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x048 (R/W)  Function Register 2                       <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L747" class="blob-num js-line-number" data-line-number="747"></td>
        <td id="LC747" class="blob-code blob-code-inner js-file-line">       <span class="pl-c1">uint32_t</span> RESERVED2[<span class="pl-c1">1</span>];</td>
      </tr>
      <tr>
        <td id="L748" class="blob-num js-line-number" data-line-number="748"></td>
        <td id="LC748" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> COMP3;                   <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x050 (R/W)  Comparator Register 3                     <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L749" class="blob-num js-line-number" data-line-number="749"></td>
        <td id="LC749" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> MASK3;                   <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x054 (R/W)  Mask Register 3                           <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L750" class="blob-num js-line-number" data-line-number="750"></td>
        <td id="LC750" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> FUNCTION3;               <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x058 (R/W)  Function Register 3                       <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L751" class="blob-num js-line-number" data-line-number="751"></td>
        <td id="LC751" class="blob-code blob-code-inner js-file-line">} DWT_Type;</td>
      </tr>
      <tr>
        <td id="L752" class="blob-num js-line-number" data-line-number="752"></td>
        <td id="LC752" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L753" class="blob-num js-line-number" data-line-number="753"></td>
        <td id="LC753" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> DWT Control Register Definitions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L754" class="blob-num js-line-number" data-line-number="754"></td>
        <td id="LC754" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_CTRL_NUMCOMP_Pos</span>               <span class="pl-c1">28</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT CTRL: NUMCOMP Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L755" class="blob-num js-line-number" data-line-number="755"></td>
        <td id="LC755" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_CTRL_NUMCOMP_Msk</span>               (0xFUL &lt;&lt; DWT_CTRL_NUMCOMP_Pos)             <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT CTRL: NUMCOMP Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L756" class="blob-num js-line-number" data-line-number="756"></td>
        <td id="LC756" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L757" class="blob-num js-line-number" data-line-number="757"></td>
        <td id="LC757" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_CTRL_NOTRCPKT_Pos</span>              <span class="pl-c1">27</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT CTRL: NOTRCPKT Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L758" class="blob-num js-line-number" data-line-number="758"></td>
        <td id="LC758" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_CTRL_NOTRCPKT_Msk</span>              (0x1UL &lt;&lt; DWT_CTRL_NOTRCPKT_Pos)            <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT CTRL: NOTRCPKT Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L759" class="blob-num js-line-number" data-line-number="759"></td>
        <td id="LC759" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L760" class="blob-num js-line-number" data-line-number="760"></td>
        <td id="LC760" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_CTRL_NOEXTTRIG_Pos</span>             <span class="pl-c1">26</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT CTRL: NOEXTTRIG Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L761" class="blob-num js-line-number" data-line-number="761"></td>
        <td id="LC761" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_CTRL_NOEXTTRIG_Msk</span>             (0x1UL &lt;&lt; DWT_CTRL_NOEXTTRIG_Pos)           <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT CTRL: NOEXTTRIG Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L762" class="blob-num js-line-number" data-line-number="762"></td>
        <td id="LC762" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L763" class="blob-num js-line-number" data-line-number="763"></td>
        <td id="LC763" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_CTRL_NOCYCCNT_Pos</span>              <span class="pl-c1">25</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT CTRL: NOCYCCNT Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L764" class="blob-num js-line-number" data-line-number="764"></td>
        <td id="LC764" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_CTRL_NOCYCCNT_Msk</span>              (0x1UL &lt;&lt; DWT_CTRL_NOCYCCNT_Pos)            <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT CTRL: NOCYCCNT Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L765" class="blob-num js-line-number" data-line-number="765"></td>
        <td id="LC765" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L766" class="blob-num js-line-number" data-line-number="766"></td>
        <td id="LC766" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_CTRL_NOPRFCNT_Pos</span>              <span class="pl-c1">24</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT CTRL: NOPRFCNT Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L767" class="blob-num js-line-number" data-line-number="767"></td>
        <td id="LC767" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_CTRL_NOPRFCNT_Msk</span>              (0x1UL &lt;&lt; DWT_CTRL_NOPRFCNT_Pos)            <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT CTRL: NOPRFCNT Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L768" class="blob-num js-line-number" data-line-number="768"></td>
        <td id="LC768" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L769" class="blob-num js-line-number" data-line-number="769"></td>
        <td id="LC769" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_CTRL_CYCEVTENA_Pos</span>             <span class="pl-c1">22</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT CTRL: CYCEVTENA Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L770" class="blob-num js-line-number" data-line-number="770"></td>
        <td id="LC770" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_CTRL_CYCEVTENA_Msk</span>             (0x1UL &lt;&lt; DWT_CTRL_CYCEVTENA_Pos)           <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT CTRL: CYCEVTENA Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L771" class="blob-num js-line-number" data-line-number="771"></td>
        <td id="LC771" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L772" class="blob-num js-line-number" data-line-number="772"></td>
        <td id="LC772" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_CTRL_FOLDEVTENA_Pos</span>            <span class="pl-c1">21</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT CTRL: FOLDEVTENA Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L773" class="blob-num js-line-number" data-line-number="773"></td>
        <td id="LC773" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_CTRL_FOLDEVTENA_Msk</span>            (0x1UL &lt;&lt; DWT_CTRL_FOLDEVTENA_Pos)          <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT CTRL: FOLDEVTENA Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L774" class="blob-num js-line-number" data-line-number="774"></td>
        <td id="LC774" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L775" class="blob-num js-line-number" data-line-number="775"></td>
        <td id="LC775" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_CTRL_LSUEVTENA_Pos</span>             <span class="pl-c1">20</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT CTRL: LSUEVTENA Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L776" class="blob-num js-line-number" data-line-number="776"></td>
        <td id="LC776" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_CTRL_LSUEVTENA_Msk</span>             (0x1UL &lt;&lt; DWT_CTRL_LSUEVTENA_Pos)           <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT CTRL: LSUEVTENA Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L777" class="blob-num js-line-number" data-line-number="777"></td>
        <td id="LC777" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L778" class="blob-num js-line-number" data-line-number="778"></td>
        <td id="LC778" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_CTRL_SLEEPEVTENA_Pos</span>           <span class="pl-c1">19</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT CTRL: SLEEPEVTENA Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L779" class="blob-num js-line-number" data-line-number="779"></td>
        <td id="LC779" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_CTRL_SLEEPEVTENA_Msk</span>           (0x1UL &lt;&lt; DWT_CTRL_SLEEPEVTENA_Pos)         <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT CTRL: SLEEPEVTENA Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L780" class="blob-num js-line-number" data-line-number="780"></td>
        <td id="LC780" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L781" class="blob-num js-line-number" data-line-number="781"></td>
        <td id="LC781" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_CTRL_EXCEVTENA_Pos</span>             <span class="pl-c1">18</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT CTRL: EXCEVTENA Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L782" class="blob-num js-line-number" data-line-number="782"></td>
        <td id="LC782" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_CTRL_EXCEVTENA_Msk</span>             (0x1UL &lt;&lt; DWT_CTRL_EXCEVTENA_Pos)           <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT CTRL: EXCEVTENA Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L783" class="blob-num js-line-number" data-line-number="783"></td>
        <td id="LC783" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L784" class="blob-num js-line-number" data-line-number="784"></td>
        <td id="LC784" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_CTRL_CPIEVTENA_Pos</span>             <span class="pl-c1">17</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT CTRL: CPIEVTENA Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L785" class="blob-num js-line-number" data-line-number="785"></td>
        <td id="LC785" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_CTRL_CPIEVTENA_Msk</span>             (0x1UL &lt;&lt; DWT_CTRL_CPIEVTENA_Pos)           <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT CTRL: CPIEVTENA Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L786" class="blob-num js-line-number" data-line-number="786"></td>
        <td id="LC786" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L787" class="blob-num js-line-number" data-line-number="787"></td>
        <td id="LC787" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_CTRL_EXCTRCENA_Pos</span>             <span class="pl-c1">16</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT CTRL: EXCTRCENA Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L788" class="blob-num js-line-number" data-line-number="788"></td>
        <td id="LC788" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_CTRL_EXCTRCENA_Msk</span>             (0x1UL &lt;&lt; DWT_CTRL_EXCTRCENA_Pos)           <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT CTRL: EXCTRCENA Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L789" class="blob-num js-line-number" data-line-number="789"></td>
        <td id="LC789" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L790" class="blob-num js-line-number" data-line-number="790"></td>
        <td id="LC790" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_CTRL_PCSAMPLENA_Pos</span>            <span class="pl-c1">12</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT CTRL: PCSAMPLENA Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L791" class="blob-num js-line-number" data-line-number="791"></td>
        <td id="LC791" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_CTRL_PCSAMPLENA_Msk</span>            (0x1UL &lt;&lt; DWT_CTRL_PCSAMPLENA_Pos)          <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT CTRL: PCSAMPLENA Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L792" class="blob-num js-line-number" data-line-number="792"></td>
        <td id="LC792" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L793" class="blob-num js-line-number" data-line-number="793"></td>
        <td id="LC793" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_CTRL_SYNCTAP_Pos</span>               <span class="pl-c1">10</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT CTRL: SYNCTAP Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L794" class="blob-num js-line-number" data-line-number="794"></td>
        <td id="LC794" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_CTRL_SYNCTAP_Msk</span>               (0x3UL &lt;&lt; DWT_CTRL_SYNCTAP_Pos)             <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT CTRL: SYNCTAP Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L795" class="blob-num js-line-number" data-line-number="795"></td>
        <td id="LC795" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L796" class="blob-num js-line-number" data-line-number="796"></td>
        <td id="LC796" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_CTRL_CYCTAP_Pos</span>                 <span class="pl-c1">9</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT CTRL: CYCTAP Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L797" class="blob-num js-line-number" data-line-number="797"></td>
        <td id="LC797" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_CTRL_CYCTAP_Msk</span>                (0x1UL &lt;&lt; DWT_CTRL_CYCTAP_Pos)              <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT CTRL: CYCTAP Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L798" class="blob-num js-line-number" data-line-number="798"></td>
        <td id="LC798" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L799" class="blob-num js-line-number" data-line-number="799"></td>
        <td id="LC799" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_CTRL_POSTINIT_Pos</span>               <span class="pl-c1">5</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT CTRL: POSTINIT Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L800" class="blob-num js-line-number" data-line-number="800"></td>
        <td id="LC800" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_CTRL_POSTINIT_Msk</span>              (0xFUL &lt;&lt; DWT_CTRL_POSTINIT_Pos)            <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT CTRL: POSTINIT Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L801" class="blob-num js-line-number" data-line-number="801"></td>
        <td id="LC801" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L802" class="blob-num js-line-number" data-line-number="802"></td>
        <td id="LC802" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_CTRL_POSTPRESET_Pos</span>             <span class="pl-c1">1</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT CTRL: POSTPRESET Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L803" class="blob-num js-line-number" data-line-number="803"></td>
        <td id="LC803" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_CTRL_POSTPRESET_Msk</span>            (0xFUL &lt;&lt; DWT_CTRL_POSTPRESET_Pos)          <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT CTRL: POSTPRESET Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L804" class="blob-num js-line-number" data-line-number="804"></td>
        <td id="LC804" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L805" class="blob-num js-line-number" data-line-number="805"></td>
        <td id="LC805" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_CTRL_CYCCNTENA_Pos</span>              <span class="pl-c1">0</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT CTRL: CYCCNTENA Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L806" class="blob-num js-line-number" data-line-number="806"></td>
        <td id="LC806" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_CTRL_CYCCNTENA_Msk</span>             (0x1UL &lt;&lt; DWT_CTRL_CYCCNTENA_Pos)           <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT CTRL: CYCCNTENA Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L807" class="blob-num js-line-number" data-line-number="807"></td>
        <td id="LC807" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L808" class="blob-num js-line-number" data-line-number="808"></td>
        <td id="LC808" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> DWT CPI Count Register Definitions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L809" class="blob-num js-line-number" data-line-number="809"></td>
        <td id="LC809" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_CPICNT_CPICNT_Pos</span>               <span class="pl-c1">0</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT CPICNT: CPICNT Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L810" class="blob-num js-line-number" data-line-number="810"></td>
        <td id="LC810" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_CPICNT_CPICNT_Msk</span>              (0xFFUL &lt;&lt; DWT_CPICNT_CPICNT_Pos)           <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT CPICNT: CPICNT Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L811" class="blob-num js-line-number" data-line-number="811"></td>
        <td id="LC811" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L812" class="blob-num js-line-number" data-line-number="812"></td>
        <td id="LC812" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> DWT Exception Overhead Count Register Definitions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L813" class="blob-num js-line-number" data-line-number="813"></td>
        <td id="LC813" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_EXCCNT_EXCCNT_Pos</span>               <span class="pl-c1">0</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT EXCCNT: EXCCNT Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L814" class="blob-num js-line-number" data-line-number="814"></td>
        <td id="LC814" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_EXCCNT_EXCCNT_Msk</span>              (0xFFUL &lt;&lt; DWT_EXCCNT_EXCCNT_Pos)           <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT EXCCNT: EXCCNT Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L815" class="blob-num js-line-number" data-line-number="815"></td>
        <td id="LC815" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L816" class="blob-num js-line-number" data-line-number="816"></td>
        <td id="LC816" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> DWT Sleep Count Register Definitions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L817" class="blob-num js-line-number" data-line-number="817"></td>
        <td id="LC817" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_SLEEPCNT_SLEEPCNT_Pos</span>           <span class="pl-c1">0</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT SLEEPCNT: SLEEPCNT Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L818" class="blob-num js-line-number" data-line-number="818"></td>
        <td id="LC818" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_SLEEPCNT_SLEEPCNT_Msk</span>          (0xFFUL &lt;&lt; DWT_SLEEPCNT_SLEEPCNT_Pos)       <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT SLEEPCNT: SLEEPCNT Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L819" class="blob-num js-line-number" data-line-number="819"></td>
        <td id="LC819" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L820" class="blob-num js-line-number" data-line-number="820"></td>
        <td id="LC820" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> DWT LSU Count Register Definitions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L821" class="blob-num js-line-number" data-line-number="821"></td>
        <td id="LC821" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_LSUCNT_LSUCNT_Pos</span>               <span class="pl-c1">0</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT LSUCNT: LSUCNT Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L822" class="blob-num js-line-number" data-line-number="822"></td>
        <td id="LC822" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_LSUCNT_LSUCNT_Msk</span>              (0xFFUL &lt;&lt; DWT_LSUCNT_LSUCNT_Pos)           <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT LSUCNT: LSUCNT Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L823" class="blob-num js-line-number" data-line-number="823"></td>
        <td id="LC823" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L824" class="blob-num js-line-number" data-line-number="824"></td>
        <td id="LC824" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> DWT Folded-instruction Count Register Definitions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L825" class="blob-num js-line-number" data-line-number="825"></td>
        <td id="LC825" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_FOLDCNT_FOLDCNT_Pos</span>             <span class="pl-c1">0</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT FOLDCNT: FOLDCNT Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L826" class="blob-num js-line-number" data-line-number="826"></td>
        <td id="LC826" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_FOLDCNT_FOLDCNT_Msk</span>            (0xFFUL &lt;&lt; DWT_FOLDCNT_FOLDCNT_Pos)         <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT FOLDCNT: FOLDCNT Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L827" class="blob-num js-line-number" data-line-number="827"></td>
        <td id="LC827" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L828" class="blob-num js-line-number" data-line-number="828"></td>
        <td id="LC828" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> DWT Comparator Mask Register Definitions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L829" class="blob-num js-line-number" data-line-number="829"></td>
        <td id="LC829" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_MASK_MASK_Pos</span>                   <span class="pl-c1">0</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT MASK: MASK Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L830" class="blob-num js-line-number" data-line-number="830"></td>
        <td id="LC830" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_MASK_MASK_Msk</span>                  (0x1FUL &lt;&lt; DWT_MASK_MASK_Pos)               <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT MASK: MASK Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L831" class="blob-num js-line-number" data-line-number="831"></td>
        <td id="LC831" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L832" class="blob-num js-line-number" data-line-number="832"></td>
        <td id="LC832" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> DWT Comparator Function Register Definitions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L833" class="blob-num js-line-number" data-line-number="833"></td>
        <td id="LC833" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_FUNCTION_MATCHED_Pos</span>           <span class="pl-c1">24</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT FUNCTION: MATCHED Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L834" class="blob-num js-line-number" data-line-number="834"></td>
        <td id="LC834" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_FUNCTION_MATCHED_Msk</span>           (0x1UL &lt;&lt; DWT_FUNCTION_MATCHED_Pos)         <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT FUNCTION: MATCHED Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L835" class="blob-num js-line-number" data-line-number="835"></td>
        <td id="LC835" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L836" class="blob-num js-line-number" data-line-number="836"></td>
        <td id="LC836" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_FUNCTION_DATAVADDR1_Pos</span>        <span class="pl-c1">16</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT FUNCTION: DATAVADDR1 Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L837" class="blob-num js-line-number" data-line-number="837"></td>
        <td id="LC837" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_FUNCTION_DATAVADDR1_Msk</span>        (0xFUL &lt;&lt; DWT_FUNCTION_DATAVADDR1_Pos)      <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT FUNCTION: DATAVADDR1 Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L838" class="blob-num js-line-number" data-line-number="838"></td>
        <td id="LC838" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L839" class="blob-num js-line-number" data-line-number="839"></td>
        <td id="LC839" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_FUNCTION_DATAVADDR0_Pos</span>        <span class="pl-c1">12</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT FUNCTION: DATAVADDR0 Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L840" class="blob-num js-line-number" data-line-number="840"></td>
        <td id="LC840" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_FUNCTION_DATAVADDR0_Msk</span>        (0xFUL &lt;&lt; DWT_FUNCTION_DATAVADDR0_Pos)      <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT FUNCTION: DATAVADDR0 Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L841" class="blob-num js-line-number" data-line-number="841"></td>
        <td id="LC841" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L842" class="blob-num js-line-number" data-line-number="842"></td>
        <td id="LC842" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_FUNCTION_DATAVSIZE_Pos</span>         <span class="pl-c1">10</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT FUNCTION: DATAVSIZE Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L843" class="blob-num js-line-number" data-line-number="843"></td>
        <td id="LC843" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_FUNCTION_DATAVSIZE_Msk</span>         (0x3UL &lt;&lt; DWT_FUNCTION_DATAVSIZE_Pos)       <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT FUNCTION: DATAVSIZE Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L844" class="blob-num js-line-number" data-line-number="844"></td>
        <td id="LC844" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L845" class="blob-num js-line-number" data-line-number="845"></td>
        <td id="LC845" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_FUNCTION_LNK1ENA_Pos</span>            <span class="pl-c1">9</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT FUNCTION: LNK1ENA Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L846" class="blob-num js-line-number" data-line-number="846"></td>
        <td id="LC846" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_FUNCTION_LNK1ENA_Msk</span>           (0x1UL &lt;&lt; DWT_FUNCTION_LNK1ENA_Pos)         <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT FUNCTION: LNK1ENA Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L847" class="blob-num js-line-number" data-line-number="847"></td>
        <td id="LC847" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L848" class="blob-num js-line-number" data-line-number="848"></td>
        <td id="LC848" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_FUNCTION_DATAVMATCH_Pos</span>         <span class="pl-c1">8</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT FUNCTION: DATAVMATCH Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L849" class="blob-num js-line-number" data-line-number="849"></td>
        <td id="LC849" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_FUNCTION_DATAVMATCH_Msk</span>        (0x1UL &lt;&lt; DWT_FUNCTION_DATAVMATCH_Pos)      <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT FUNCTION: DATAVMATCH Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L850" class="blob-num js-line-number" data-line-number="850"></td>
        <td id="LC850" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L851" class="blob-num js-line-number" data-line-number="851"></td>
        <td id="LC851" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_FUNCTION_CYCMATCH_Pos</span>           <span class="pl-c1">7</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT FUNCTION: CYCMATCH Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L852" class="blob-num js-line-number" data-line-number="852"></td>
        <td id="LC852" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_FUNCTION_CYCMATCH_Msk</span>          (0x1UL &lt;&lt; DWT_FUNCTION_CYCMATCH_Pos)        <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT FUNCTION: CYCMATCH Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L853" class="blob-num js-line-number" data-line-number="853"></td>
        <td id="LC853" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L854" class="blob-num js-line-number" data-line-number="854"></td>
        <td id="LC854" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_FUNCTION_EMITRANGE_Pos</span>          <span class="pl-c1">5</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT FUNCTION: EMITRANGE Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L855" class="blob-num js-line-number" data-line-number="855"></td>
        <td id="LC855" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_FUNCTION_EMITRANGE_Msk</span>         (0x1UL &lt;&lt; DWT_FUNCTION_EMITRANGE_Pos)       <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT FUNCTION: EMITRANGE Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L856" class="blob-num js-line-number" data-line-number="856"></td>
        <td id="LC856" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L857" class="blob-num js-line-number" data-line-number="857"></td>
        <td id="LC857" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_FUNCTION_FUNCTION_Pos</span>           <span class="pl-c1">0</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT FUNCTION: FUNCTION Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L858" class="blob-num js-line-number" data-line-number="858"></td>
        <td id="LC858" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_FUNCTION_FUNCTION_Msk</span>          (0xFUL &lt;&lt; DWT_FUNCTION_FUNCTION_Pos)        <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT FUNCTION: FUNCTION Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L859" class="blob-num js-line-number" data-line-number="859"></td>
        <td id="LC859" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L860" class="blob-num js-line-number" data-line-number="860"></td>
        <td id="LC860" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>@}<span class="pl-c">*/</span></span> <span class="pl-c"><span class="pl-c">/*</span> end of group CMSIS_DWT <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L861" class="blob-num js-line-number" data-line-number="861"></td>
        <td id="LC861" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L862" class="blob-num js-line-number" data-line-number="862"></td>
        <td id="LC862" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L863" class="blob-num js-line-number" data-line-number="863"></td>
        <td id="LC863" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \ingroup  CMSIS_core_register</span></td>
      </tr>
      <tr>
        <td id="L864" class="blob-num js-line-number" data-line-number="864"></td>
        <td id="LC864" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \defgroup CMSIS_TPI     Trace Port Interface (TPI)</span></td>
      </tr>
      <tr>
        <td id="L865" class="blob-num js-line-number" data-line-number="865"></td>
        <td id="LC865" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \brief      Type definitions for the Trace Port Interface (TPI)</span></td>
      </tr>
      <tr>
        <td id="L866" class="blob-num js-line-number" data-line-number="866"></td>
        <td id="LC866" class="blob-code blob-code-inner js-file-line"><span class="pl-c">  @{</span></td>
      </tr>
      <tr>
        <td id="L867" class="blob-num js-line-number" data-line-number="867"></td>
        <td id="LC867" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L868" class="blob-num js-line-number" data-line-number="868"></td>
        <td id="LC868" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L869" class="blob-num js-line-number" data-line-number="869"></td>
        <td id="LC869" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \brief  Structure type to access the Trace Port Interface Register (TPI).</span></td>
      </tr>
      <tr>
        <td id="L870" class="blob-num js-line-number" data-line-number="870"></td>
        <td id="LC870" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L871" class="blob-num js-line-number" data-line-number="871"></td>
        <td id="LC871" class="blob-code blob-code-inner js-file-line"><span class="pl-k">typedef</span> <span class="pl-k">struct</span></td>
      </tr>
      <tr>
        <td id="L872" class="blob-num js-line-number" data-line-number="872"></td>
        <td id="LC872" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L873" class="blob-num js-line-number" data-line-number="873"></td>
        <td id="LC873" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> SSPSR;                   <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x000 (R/ )  Supported Parallel Port Size Register     <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L874" class="blob-num js-line-number" data-line-number="874"></td>
        <td id="LC874" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> CSPSR;                   <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x004 (R/W)  Current Parallel Port Size Register <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L875" class="blob-num js-line-number" data-line-number="875"></td>
        <td id="LC875" class="blob-code blob-code-inner js-file-line">       <span class="pl-c1">uint32_t</span> RESERVED0[<span class="pl-c1">2</span>];</td>
      </tr>
      <tr>
        <td id="L876" class="blob-num js-line-number" data-line-number="876"></td>
        <td id="LC876" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> ACPR;                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x010 (R/W)  Asynchronous Clock Prescaler Register <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L877" class="blob-num js-line-number" data-line-number="877"></td>
        <td id="LC877" class="blob-code blob-code-inner js-file-line">       <span class="pl-c1">uint32_t</span> RESERVED1[<span class="pl-c1">55</span>];</td>
      </tr>
      <tr>
        <td id="L878" class="blob-num js-line-number" data-line-number="878"></td>
        <td id="LC878" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> SPPR;                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x0F0 (R/W)  Selected Pin Protocol Register <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L879" class="blob-num js-line-number" data-line-number="879"></td>
        <td id="LC879" class="blob-code blob-code-inner js-file-line">       <span class="pl-c1">uint32_t</span> RESERVED2[<span class="pl-c1">131</span>];</td>
      </tr>
      <tr>
        <td id="L880" class="blob-num js-line-number" data-line-number="880"></td>
        <td id="LC880" class="blob-code blob-code-inner js-file-line">  __I  <span class="pl-c1">uint32_t</span> FFSR;                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x300 (R/ )  Formatter and Flush Status Register <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L881" class="blob-num js-line-number" data-line-number="881"></td>
        <td id="LC881" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> FFCR;                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x304 (R/W)  Formatter and Flush Control Register <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L882" class="blob-num js-line-number" data-line-number="882"></td>
        <td id="LC882" class="blob-code blob-code-inner js-file-line">  __I  <span class="pl-c1">uint32_t</span> FSCR;                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x308 (R/ )  Formatter Synchronization Counter Register <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L883" class="blob-num js-line-number" data-line-number="883"></td>
        <td id="LC883" class="blob-code blob-code-inner js-file-line">       <span class="pl-c1">uint32_t</span> RESERVED3[<span class="pl-c1">759</span>];</td>
      </tr>
      <tr>
        <td id="L884" class="blob-num js-line-number" data-line-number="884"></td>
        <td id="LC884" class="blob-code blob-code-inner js-file-line">  __I  <span class="pl-c1">uint32_t</span> TRIGGER;                 <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0xEE8 (R/ )  TRIGGER <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L885" class="blob-num js-line-number" data-line-number="885"></td>
        <td id="LC885" class="blob-code blob-code-inner js-file-line">  __I  <span class="pl-c1">uint32_t</span> FIFO0;                   <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0xEEC (R/ )  Integration ETM Data <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L886" class="blob-num js-line-number" data-line-number="886"></td>
        <td id="LC886" class="blob-code blob-code-inner js-file-line">  __I  <span class="pl-c1">uint32_t</span> ITATBCTR2;               <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0xEF0 (R/ )  ITATBCTR2 <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L887" class="blob-num js-line-number" data-line-number="887"></td>
        <td id="LC887" class="blob-code blob-code-inner js-file-line">       <span class="pl-c1">uint32_t</span> RESERVED4[<span class="pl-c1">1</span>];</td>
      </tr>
      <tr>
        <td id="L888" class="blob-num js-line-number" data-line-number="888"></td>
        <td id="LC888" class="blob-code blob-code-inner js-file-line">  __I  <span class="pl-c1">uint32_t</span> ITATBCTR0;               <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0xEF8 (R/ )  ITATBCTR0 <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L889" class="blob-num js-line-number" data-line-number="889"></td>
        <td id="LC889" class="blob-code blob-code-inner js-file-line">  __I  <span class="pl-c1">uint32_t</span> FIFO1;                   <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0xEFC (R/ )  Integration ITM Data <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L890" class="blob-num js-line-number" data-line-number="890"></td>
        <td id="LC890" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> ITCTRL;                  <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0xF00 (R/W)  Integration Mode Control <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L891" class="blob-num js-line-number" data-line-number="891"></td>
        <td id="LC891" class="blob-code blob-code-inner js-file-line">       <span class="pl-c1">uint32_t</span> RESERVED5[<span class="pl-c1">39</span>];</td>
      </tr>
      <tr>
        <td id="L892" class="blob-num js-line-number" data-line-number="892"></td>
        <td id="LC892" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> CLAIMSET;                <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0xFA0 (R/W)  Claim tag set <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L893" class="blob-num js-line-number" data-line-number="893"></td>
        <td id="LC893" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> CLAIMCLR;                <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0xFA4 (R/W)  Claim tag clear <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L894" class="blob-num js-line-number" data-line-number="894"></td>
        <td id="LC894" class="blob-code blob-code-inner js-file-line">       <span class="pl-c1">uint32_t</span> RESERVED7[<span class="pl-c1">8</span>];</td>
      </tr>
      <tr>
        <td id="L895" class="blob-num js-line-number" data-line-number="895"></td>
        <td id="LC895" class="blob-code blob-code-inner js-file-line">  __I  <span class="pl-c1">uint32_t</span> DEVID;                   <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0xFC8 (R/ )  TPIU_DEVID <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L896" class="blob-num js-line-number" data-line-number="896"></td>
        <td id="LC896" class="blob-code blob-code-inner js-file-line">  __I  <span class="pl-c1">uint32_t</span> DEVTYPE;                 <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0xFCC (R/ )  TPIU_DEVTYPE <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L897" class="blob-num js-line-number" data-line-number="897"></td>
        <td id="LC897" class="blob-code blob-code-inner js-file-line">} TPI_Type;</td>
      </tr>
      <tr>
        <td id="L898" class="blob-num js-line-number" data-line-number="898"></td>
        <td id="LC898" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L899" class="blob-num js-line-number" data-line-number="899"></td>
        <td id="LC899" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> TPI Asynchronous Clock Prescaler Register Definitions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L900" class="blob-num js-line-number" data-line-number="900"></td>
        <td id="LC900" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_ACPR_PRESCALER_Pos</span>              <span class="pl-c1">0</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI ACPR: PRESCALER Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L901" class="blob-num js-line-number" data-line-number="901"></td>
        <td id="LC901" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_ACPR_PRESCALER_Msk</span>             (0x1FFFUL &lt;&lt; TPI_ACPR_PRESCALER_Pos)        <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI ACPR: PRESCALER Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L902" class="blob-num js-line-number" data-line-number="902"></td>
        <td id="LC902" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L903" class="blob-num js-line-number" data-line-number="903"></td>
        <td id="LC903" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> TPI Selected Pin Protocol Register Definitions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L904" class="blob-num js-line-number" data-line-number="904"></td>
        <td id="LC904" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_SPPR_TXMODE_Pos</span>                 <span class="pl-c1">0</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI SPPR: TXMODE Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L905" class="blob-num js-line-number" data-line-number="905"></td>
        <td id="LC905" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_SPPR_TXMODE_Msk</span>                (0x3UL &lt;&lt; TPI_SPPR_TXMODE_Pos)              <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI SPPR: TXMODE Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L906" class="blob-num js-line-number" data-line-number="906"></td>
        <td id="LC906" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L907" class="blob-num js-line-number" data-line-number="907"></td>
        <td id="LC907" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> TPI Formatter and Flush Status Register Definitions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L908" class="blob-num js-line-number" data-line-number="908"></td>
        <td id="LC908" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_FFSR_FtNonStop_Pos</span>              <span class="pl-c1">3</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI FFSR: FtNonStop Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L909" class="blob-num js-line-number" data-line-number="909"></td>
        <td id="LC909" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_FFSR_FtNonStop_Msk</span>             (0x1UL &lt;&lt; TPI_FFSR_FtNonStop_Pos)           <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI FFSR: FtNonStop Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L910" class="blob-num js-line-number" data-line-number="910"></td>
        <td id="LC910" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L911" class="blob-num js-line-number" data-line-number="911"></td>
        <td id="LC911" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_FFSR_TCPresent_Pos</span>              <span class="pl-c1">2</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI FFSR: TCPresent Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L912" class="blob-num js-line-number" data-line-number="912"></td>
        <td id="LC912" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_FFSR_TCPresent_Msk</span>             (0x1UL &lt;&lt; TPI_FFSR_TCPresent_Pos)           <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI FFSR: TCPresent Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L913" class="blob-num js-line-number" data-line-number="913"></td>
        <td id="LC913" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L914" class="blob-num js-line-number" data-line-number="914"></td>
        <td id="LC914" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_FFSR_FtStopped_Pos</span>              <span class="pl-c1">1</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI FFSR: FtStopped Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L915" class="blob-num js-line-number" data-line-number="915"></td>
        <td id="LC915" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_FFSR_FtStopped_Msk</span>             (0x1UL &lt;&lt; TPI_FFSR_FtStopped_Pos)           <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI FFSR: FtStopped Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L916" class="blob-num js-line-number" data-line-number="916"></td>
        <td id="LC916" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L917" class="blob-num js-line-number" data-line-number="917"></td>
        <td id="LC917" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_FFSR_FlInProg_Pos</span>               <span class="pl-c1">0</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI FFSR: FlInProg Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L918" class="blob-num js-line-number" data-line-number="918"></td>
        <td id="LC918" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_FFSR_FlInProg_Msk</span>              (0x1UL &lt;&lt; TPI_FFSR_FlInProg_Pos)            <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI FFSR: FlInProg Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L919" class="blob-num js-line-number" data-line-number="919"></td>
        <td id="LC919" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L920" class="blob-num js-line-number" data-line-number="920"></td>
        <td id="LC920" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> TPI Formatter and Flush Control Register Definitions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L921" class="blob-num js-line-number" data-line-number="921"></td>
        <td id="LC921" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_FFCR_TrigIn_Pos</span>                 <span class="pl-c1">8</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI FFCR: TrigIn Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L922" class="blob-num js-line-number" data-line-number="922"></td>
        <td id="LC922" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_FFCR_TrigIn_Msk</span>                (0x1UL &lt;&lt; TPI_FFCR_TrigIn_Pos)              <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI FFCR: TrigIn Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L923" class="blob-num js-line-number" data-line-number="923"></td>
        <td id="LC923" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L924" class="blob-num js-line-number" data-line-number="924"></td>
        <td id="LC924" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_FFCR_EnFCont_Pos</span>                <span class="pl-c1">1</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI FFCR: EnFCont Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L925" class="blob-num js-line-number" data-line-number="925"></td>
        <td id="LC925" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_FFCR_EnFCont_Msk</span>               (0x1UL &lt;&lt; TPI_FFCR_EnFCont_Pos)             <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI FFCR: EnFCont Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L926" class="blob-num js-line-number" data-line-number="926"></td>
        <td id="LC926" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L927" class="blob-num js-line-number" data-line-number="927"></td>
        <td id="LC927" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> TPI TRIGGER Register Definitions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L928" class="blob-num js-line-number" data-line-number="928"></td>
        <td id="LC928" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_TRIGGER_TRIGGER_Pos</span>             <span class="pl-c1">0</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI TRIGGER: TRIGGER Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L929" class="blob-num js-line-number" data-line-number="929"></td>
        <td id="LC929" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_TRIGGER_TRIGGER_Msk</span>            (0x1UL &lt;&lt; TPI_TRIGGER_TRIGGER_Pos)          <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI TRIGGER: TRIGGER Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L930" class="blob-num js-line-number" data-line-number="930"></td>
        <td id="LC930" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L931" class="blob-num js-line-number" data-line-number="931"></td>
        <td id="LC931" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> TPI Integration ETM Data Register Definitions (FIFO0) <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L932" class="blob-num js-line-number" data-line-number="932"></td>
        <td id="LC932" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_FIFO0_ITM_ATVALID_Pos</span>          <span class="pl-c1">29</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI FIFO0: ITM_ATVALID Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L933" class="blob-num js-line-number" data-line-number="933"></td>
        <td id="LC933" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_FIFO0_ITM_ATVALID_Msk</span>          (0x3UL &lt;&lt; TPI_FIFO0_ITM_ATVALID_Pos)        <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI FIFO0: ITM_ATVALID Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L934" class="blob-num js-line-number" data-line-number="934"></td>
        <td id="LC934" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L935" class="blob-num js-line-number" data-line-number="935"></td>
        <td id="LC935" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_FIFO0_ITM_bytecount_Pos</span>        <span class="pl-c1">27</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI FIFO0: ITM_bytecount Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L936" class="blob-num js-line-number" data-line-number="936"></td>
        <td id="LC936" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_FIFO0_ITM_bytecount_Msk</span>        (0x3UL &lt;&lt; TPI_FIFO0_ITM_bytecount_Pos)      <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI FIFO0: ITM_bytecount Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L937" class="blob-num js-line-number" data-line-number="937"></td>
        <td id="LC937" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L938" class="blob-num js-line-number" data-line-number="938"></td>
        <td id="LC938" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_FIFO0_ETM_ATVALID_Pos</span>          <span class="pl-c1">26</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI FIFO0: ETM_ATVALID Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L939" class="blob-num js-line-number" data-line-number="939"></td>
        <td id="LC939" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_FIFO0_ETM_ATVALID_Msk</span>          (0x3UL &lt;&lt; TPI_FIFO0_ETM_ATVALID_Pos)        <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI FIFO0: ETM_ATVALID Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L940" class="blob-num js-line-number" data-line-number="940"></td>
        <td id="LC940" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L941" class="blob-num js-line-number" data-line-number="941"></td>
        <td id="LC941" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_FIFO0_ETM_bytecount_Pos</span>        <span class="pl-c1">24</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI FIFO0: ETM_bytecount Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L942" class="blob-num js-line-number" data-line-number="942"></td>
        <td id="LC942" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_FIFO0_ETM_bytecount_Msk</span>        (0x3UL &lt;&lt; TPI_FIFO0_ETM_bytecount_Pos)      <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI FIFO0: ETM_bytecount Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L943" class="blob-num js-line-number" data-line-number="943"></td>
        <td id="LC943" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L944" class="blob-num js-line-number" data-line-number="944"></td>
        <td id="LC944" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_FIFO0_ETM2_Pos</span>                 <span class="pl-c1">16</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI FIFO0: ETM2 Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L945" class="blob-num js-line-number" data-line-number="945"></td>
        <td id="LC945" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_FIFO0_ETM2_Msk</span>                 (0xFFUL &lt;&lt; TPI_FIFO0_ETM2_Pos)              <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI FIFO0: ETM2 Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L946" class="blob-num js-line-number" data-line-number="946"></td>
        <td id="LC946" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L947" class="blob-num js-line-number" data-line-number="947"></td>
        <td id="LC947" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_FIFO0_ETM1_Pos</span>                  <span class="pl-c1">8</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI FIFO0: ETM1 Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L948" class="blob-num js-line-number" data-line-number="948"></td>
        <td id="LC948" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_FIFO0_ETM1_Msk</span>                 (0xFFUL &lt;&lt; TPI_FIFO0_ETM1_Pos)              <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI FIFO0: ETM1 Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L949" class="blob-num js-line-number" data-line-number="949"></td>
        <td id="LC949" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L950" class="blob-num js-line-number" data-line-number="950"></td>
        <td id="LC950" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_FIFO0_ETM0_Pos</span>                  <span class="pl-c1">0</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI FIFO0: ETM0 Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L951" class="blob-num js-line-number" data-line-number="951"></td>
        <td id="LC951" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_FIFO0_ETM0_Msk</span>                 (0xFFUL &lt;&lt; TPI_FIFO0_ETM0_Pos)              <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI FIFO0: ETM0 Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L952" class="blob-num js-line-number" data-line-number="952"></td>
        <td id="LC952" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L953" class="blob-num js-line-number" data-line-number="953"></td>
        <td id="LC953" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> TPI ITATBCTR2 Register Definitions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L954" class="blob-num js-line-number" data-line-number="954"></td>
        <td id="LC954" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_ITATBCTR2_ATREADY_Pos</span>           <span class="pl-c1">0</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI ITATBCTR2: ATREADY Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L955" class="blob-num js-line-number" data-line-number="955"></td>
        <td id="LC955" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_ITATBCTR2_ATREADY_Msk</span>          (0x1UL &lt;&lt; TPI_ITATBCTR2_ATREADY_Pos)        <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI ITATBCTR2: ATREADY Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L956" class="blob-num js-line-number" data-line-number="956"></td>
        <td id="LC956" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L957" class="blob-num js-line-number" data-line-number="957"></td>
        <td id="LC957" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> TPI Integration ITM Data Register Definitions (FIFO1) <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L958" class="blob-num js-line-number" data-line-number="958"></td>
        <td id="LC958" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_FIFO1_ITM_ATVALID_Pos</span>          <span class="pl-c1">29</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI FIFO1: ITM_ATVALID Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L959" class="blob-num js-line-number" data-line-number="959"></td>
        <td id="LC959" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_FIFO1_ITM_ATVALID_Msk</span>          (0x3UL &lt;&lt; TPI_FIFO1_ITM_ATVALID_Pos)        <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI FIFO1: ITM_ATVALID Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L960" class="blob-num js-line-number" data-line-number="960"></td>
        <td id="LC960" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L961" class="blob-num js-line-number" data-line-number="961"></td>
        <td id="LC961" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_FIFO1_ITM_bytecount_Pos</span>        <span class="pl-c1">27</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI FIFO1: ITM_bytecount Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L962" class="blob-num js-line-number" data-line-number="962"></td>
        <td id="LC962" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_FIFO1_ITM_bytecount_Msk</span>        (0x3UL &lt;&lt; TPI_FIFO1_ITM_bytecount_Pos)      <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI FIFO1: ITM_bytecount Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L963" class="blob-num js-line-number" data-line-number="963"></td>
        <td id="LC963" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L964" class="blob-num js-line-number" data-line-number="964"></td>
        <td id="LC964" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_FIFO1_ETM_ATVALID_Pos</span>          <span class="pl-c1">26</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI FIFO1: ETM_ATVALID Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L965" class="blob-num js-line-number" data-line-number="965"></td>
        <td id="LC965" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_FIFO1_ETM_ATVALID_Msk</span>          (0x3UL &lt;&lt; TPI_FIFO1_ETM_ATVALID_Pos)        <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI FIFO1: ETM_ATVALID Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L966" class="blob-num js-line-number" data-line-number="966"></td>
        <td id="LC966" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L967" class="blob-num js-line-number" data-line-number="967"></td>
        <td id="LC967" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_FIFO1_ETM_bytecount_Pos</span>        <span class="pl-c1">24</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI FIFO1: ETM_bytecount Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L968" class="blob-num js-line-number" data-line-number="968"></td>
        <td id="LC968" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_FIFO1_ETM_bytecount_Msk</span>        (0x3UL &lt;&lt; TPI_FIFO1_ETM_bytecount_Pos)      <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI FIFO1: ETM_bytecount Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L969" class="blob-num js-line-number" data-line-number="969"></td>
        <td id="LC969" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L970" class="blob-num js-line-number" data-line-number="970"></td>
        <td id="LC970" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_FIFO1_ITM2_Pos</span>                 <span class="pl-c1">16</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI FIFO1: ITM2 Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L971" class="blob-num js-line-number" data-line-number="971"></td>
        <td id="LC971" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_FIFO1_ITM2_Msk</span>                 (0xFFUL &lt;&lt; TPI_FIFO1_ITM2_Pos)              <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI FIFO1: ITM2 Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L972" class="blob-num js-line-number" data-line-number="972"></td>
        <td id="LC972" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L973" class="blob-num js-line-number" data-line-number="973"></td>
        <td id="LC973" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_FIFO1_ITM1_Pos</span>                  <span class="pl-c1">8</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI FIFO1: ITM1 Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L974" class="blob-num js-line-number" data-line-number="974"></td>
        <td id="LC974" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_FIFO1_ITM1_Msk</span>                 (0xFFUL &lt;&lt; TPI_FIFO1_ITM1_Pos)              <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI FIFO1: ITM1 Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L975" class="blob-num js-line-number" data-line-number="975"></td>
        <td id="LC975" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L976" class="blob-num js-line-number" data-line-number="976"></td>
        <td id="LC976" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_FIFO1_ITM0_Pos</span>                  <span class="pl-c1">0</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI FIFO1: ITM0 Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L977" class="blob-num js-line-number" data-line-number="977"></td>
        <td id="LC977" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_FIFO1_ITM0_Msk</span>                 (0xFFUL &lt;&lt; TPI_FIFO1_ITM0_Pos)              <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI FIFO1: ITM0 Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L978" class="blob-num js-line-number" data-line-number="978"></td>
        <td id="LC978" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L979" class="blob-num js-line-number" data-line-number="979"></td>
        <td id="LC979" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> TPI ITATBCTR0 Register Definitions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L980" class="blob-num js-line-number" data-line-number="980"></td>
        <td id="LC980" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_ITATBCTR0_ATREADY_Pos</span>           <span class="pl-c1">0</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI ITATBCTR0: ATREADY Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L981" class="blob-num js-line-number" data-line-number="981"></td>
        <td id="LC981" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_ITATBCTR0_ATREADY_Msk</span>          (0x1UL &lt;&lt; TPI_ITATBCTR0_ATREADY_Pos)        <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI ITATBCTR0: ATREADY Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L982" class="blob-num js-line-number" data-line-number="982"></td>
        <td id="LC982" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L983" class="blob-num js-line-number" data-line-number="983"></td>
        <td id="LC983" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> TPI Integration Mode Control Register Definitions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L984" class="blob-num js-line-number" data-line-number="984"></td>
        <td id="LC984" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_ITCTRL_Mode_Pos</span>                 <span class="pl-c1">0</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI ITCTRL: Mode Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L985" class="blob-num js-line-number" data-line-number="985"></td>
        <td id="LC985" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_ITCTRL_Mode_Msk</span>                (0x1UL &lt;&lt; TPI_ITCTRL_Mode_Pos)              <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI ITCTRL: Mode Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L986" class="blob-num js-line-number" data-line-number="986"></td>
        <td id="LC986" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L987" class="blob-num js-line-number" data-line-number="987"></td>
        <td id="LC987" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> TPI DEVID Register Definitions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L988" class="blob-num js-line-number" data-line-number="988"></td>
        <td id="LC988" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_DEVID_NRZVALID_Pos</span>             <span class="pl-c1">11</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI DEVID: NRZVALID Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L989" class="blob-num js-line-number" data-line-number="989"></td>
        <td id="LC989" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_DEVID_NRZVALID_Msk</span>             (0x1UL &lt;&lt; TPI_DEVID_NRZVALID_Pos)           <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI DEVID: NRZVALID Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L990" class="blob-num js-line-number" data-line-number="990"></td>
        <td id="LC990" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L991" class="blob-num js-line-number" data-line-number="991"></td>
        <td id="LC991" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_DEVID_MANCVALID_Pos</span>            <span class="pl-c1">10</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI DEVID: MANCVALID Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L992" class="blob-num js-line-number" data-line-number="992"></td>
        <td id="LC992" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_DEVID_MANCVALID_Msk</span>            (0x1UL &lt;&lt; TPI_DEVID_MANCVALID_Pos)          <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI DEVID: MANCVALID Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L993" class="blob-num js-line-number" data-line-number="993"></td>
        <td id="LC993" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L994" class="blob-num js-line-number" data-line-number="994"></td>
        <td id="LC994" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_DEVID_PTINVALID_Pos</span>             <span class="pl-c1">9</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI DEVID: PTINVALID Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L995" class="blob-num js-line-number" data-line-number="995"></td>
        <td id="LC995" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_DEVID_PTINVALID_Msk</span>            (0x1UL &lt;&lt; TPI_DEVID_PTINVALID_Pos)          <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI DEVID: PTINVALID Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L996" class="blob-num js-line-number" data-line-number="996"></td>
        <td id="LC996" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L997" class="blob-num js-line-number" data-line-number="997"></td>
        <td id="LC997" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_DEVID_MinBufSz_Pos</span>              <span class="pl-c1">6</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI DEVID: MinBufSz Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L998" class="blob-num js-line-number" data-line-number="998"></td>
        <td id="LC998" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_DEVID_MinBufSz_Msk</span>             (0x7UL &lt;&lt; TPI_DEVID_MinBufSz_Pos)           <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI DEVID: MinBufSz Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L999" class="blob-num js-line-number" data-line-number="999"></td>
        <td id="LC999" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1000" class="blob-num js-line-number" data-line-number="1000"></td>
        <td id="LC1000" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_DEVID_AsynClkIn_Pos</span>             <span class="pl-c1">5</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI DEVID: AsynClkIn Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1001" class="blob-num js-line-number" data-line-number="1001"></td>
        <td id="LC1001" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_DEVID_AsynClkIn_Msk</span>            (0x1UL &lt;&lt; TPI_DEVID_AsynClkIn_Pos)          <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI DEVID: AsynClkIn Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1002" class="blob-num js-line-number" data-line-number="1002"></td>
        <td id="LC1002" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1003" class="blob-num js-line-number" data-line-number="1003"></td>
        <td id="LC1003" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_DEVID_NrTraceInput_Pos</span>          <span class="pl-c1">0</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI DEVID: NrTraceInput Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1004" class="blob-num js-line-number" data-line-number="1004"></td>
        <td id="LC1004" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_DEVID_NrTraceInput_Msk</span>         (0x1FUL &lt;&lt; TPI_DEVID_NrTraceInput_Pos)      <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI DEVID: NrTraceInput Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1005" class="blob-num js-line-number" data-line-number="1005"></td>
        <td id="LC1005" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1006" class="blob-num js-line-number" data-line-number="1006"></td>
        <td id="LC1006" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> TPI DEVTYPE Register Definitions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1007" class="blob-num js-line-number" data-line-number="1007"></td>
        <td id="LC1007" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_DEVTYPE_SubType_Pos</span>             <span class="pl-c1">0</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI DEVTYPE: SubType Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1008" class="blob-num js-line-number" data-line-number="1008"></td>
        <td id="LC1008" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_DEVTYPE_SubType_Msk</span>            (0xFUL &lt;&lt; TPI_DEVTYPE_SubType_Pos)          <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI DEVTYPE: SubType Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1009" class="blob-num js-line-number" data-line-number="1009"></td>
        <td id="LC1009" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1010" class="blob-num js-line-number" data-line-number="1010"></td>
        <td id="LC1010" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_DEVTYPE_MajorType_Pos</span>           <span class="pl-c1">4</span>                                          <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI DEVTYPE: MajorType Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1011" class="blob-num js-line-number" data-line-number="1011"></td>
        <td id="LC1011" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_DEVTYPE_MajorType_Msk</span>          (0xFUL &lt;&lt; TPI_DEVTYPE_MajorType_Pos)        <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI DEVTYPE: MajorType Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1012" class="blob-num js-line-number" data-line-number="1012"></td>
        <td id="LC1012" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1013" class="blob-num js-line-number" data-line-number="1013"></td>
        <td id="LC1013" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>@}<span class="pl-c">*/</span></span> <span class="pl-c"><span class="pl-c">/*</span> end of group CMSIS_TPI <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1014" class="blob-num js-line-number" data-line-number="1014"></td>
        <td id="LC1014" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1015" class="blob-num js-line-number" data-line-number="1015"></td>
        <td id="LC1015" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1016" class="blob-num js-line-number" data-line-number="1016"></td>
        <td id="LC1016" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">if</span> (__MPU_PRESENT == 1)</td>
      </tr>
      <tr>
        <td id="L1017" class="blob-num js-line-number" data-line-number="1017"></td>
        <td id="LC1017" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \ingroup  CMSIS_core_register</span></td>
      </tr>
      <tr>
        <td id="L1018" class="blob-num js-line-number" data-line-number="1018"></td>
        <td id="LC1018" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \defgroup CMSIS_MPU     Memory Protection Unit (MPU)</span></td>
      </tr>
      <tr>
        <td id="L1019" class="blob-num js-line-number" data-line-number="1019"></td>
        <td id="LC1019" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \brief      Type definitions for the Memory Protection Unit (MPU)</span></td>
      </tr>
      <tr>
        <td id="L1020" class="blob-num js-line-number" data-line-number="1020"></td>
        <td id="LC1020" class="blob-code blob-code-inner js-file-line"><span class="pl-c">  @{</span></td>
      </tr>
      <tr>
        <td id="L1021" class="blob-num js-line-number" data-line-number="1021"></td>
        <td id="LC1021" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1022" class="blob-num js-line-number" data-line-number="1022"></td>
        <td id="LC1022" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1023" class="blob-num js-line-number" data-line-number="1023"></td>
        <td id="LC1023" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \brief  Structure type to access the Memory Protection Unit (MPU).</span></td>
      </tr>
      <tr>
        <td id="L1024" class="blob-num js-line-number" data-line-number="1024"></td>
        <td id="LC1024" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1025" class="blob-num js-line-number" data-line-number="1025"></td>
        <td id="LC1025" class="blob-code blob-code-inner js-file-line"><span class="pl-k">typedef</span> <span class="pl-k">struct</span></td>
      </tr>
      <tr>
        <td id="L1026" class="blob-num js-line-number" data-line-number="1026"></td>
        <td id="LC1026" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L1027" class="blob-num js-line-number" data-line-number="1027"></td>
        <td id="LC1027" class="blob-code blob-code-inner js-file-line">  __I  <span class="pl-c1">uint32_t</span> TYPE;                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x000 (R/ )  MPU Type Register                              <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1028" class="blob-num js-line-number" data-line-number="1028"></td>
        <td id="LC1028" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> CTRL;                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x004 (R/W)  MPU Control Register                           <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1029" class="blob-num js-line-number" data-line-number="1029"></td>
        <td id="LC1029" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> RNR;                     <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x008 (R/W)  MPU Region RNRber Register                     <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1030" class="blob-num js-line-number" data-line-number="1030"></td>
        <td id="LC1030" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> RBAR;                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x00C (R/W)  MPU Region Base Address Register               <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1031" class="blob-num js-line-number" data-line-number="1031"></td>
        <td id="LC1031" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> RASR;                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x010 (R/W)  MPU Region Attribute and Size Register         <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1032" class="blob-num js-line-number" data-line-number="1032"></td>
        <td id="LC1032" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> RBAR_A1;                 <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x014 (R/W)  MPU Alias 1 Region Base Address Register       <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1033" class="blob-num js-line-number" data-line-number="1033"></td>
        <td id="LC1033" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> RASR_A1;                 <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x018 (R/W)  MPU Alias 1 Region Attribute and Size Register <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1034" class="blob-num js-line-number" data-line-number="1034"></td>
        <td id="LC1034" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> RBAR_A2;                 <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x01C (R/W)  MPU Alias 2 Region Base Address Register       <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1035" class="blob-num js-line-number" data-line-number="1035"></td>
        <td id="LC1035" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> RASR_A2;                 <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x020 (R/W)  MPU Alias 2 Region Attribute and Size Register <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1036" class="blob-num js-line-number" data-line-number="1036"></td>
        <td id="LC1036" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> RBAR_A3;                 <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x024 (R/W)  MPU Alias 3 Region Base Address Register       <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1037" class="blob-num js-line-number" data-line-number="1037"></td>
        <td id="LC1037" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> RASR_A3;                 <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x028 (R/W)  MPU Alias 3 Region Attribute and Size Register <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1038" class="blob-num js-line-number" data-line-number="1038"></td>
        <td id="LC1038" class="blob-code blob-code-inner js-file-line">} MPU_Type;</td>
      </tr>
      <tr>
        <td id="L1039" class="blob-num js-line-number" data-line-number="1039"></td>
        <td id="LC1039" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1040" class="blob-num js-line-number" data-line-number="1040"></td>
        <td id="LC1040" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> MPU Type Register <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1041" class="blob-num js-line-number" data-line-number="1041"></td>
        <td id="LC1041" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU_TYPE_IREGION_Pos</span>               <span class="pl-c1">16</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; MPU TYPE: IREGION Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1042" class="blob-num js-line-number" data-line-number="1042"></td>
        <td id="LC1042" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU_TYPE_IREGION_Msk</span>               (0xFFUL &lt;&lt; MPU_TYPE_IREGION_Pos)               <span class="pl-c"><span class="pl-c">/*</span>!&lt; MPU TYPE: IREGION Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1043" class="blob-num js-line-number" data-line-number="1043"></td>
        <td id="LC1043" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1044" class="blob-num js-line-number" data-line-number="1044"></td>
        <td id="LC1044" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU_TYPE_DREGION_Pos</span>                <span class="pl-c1">8</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; MPU TYPE: DREGION Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1045" class="blob-num js-line-number" data-line-number="1045"></td>
        <td id="LC1045" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU_TYPE_DREGION_Msk</span>               (0xFFUL &lt;&lt; MPU_TYPE_DREGION_Pos)               <span class="pl-c"><span class="pl-c">/*</span>!&lt; MPU TYPE: DREGION Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1046" class="blob-num js-line-number" data-line-number="1046"></td>
        <td id="LC1046" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1047" class="blob-num js-line-number" data-line-number="1047"></td>
        <td id="LC1047" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU_TYPE_SEPARATE_Pos</span>               <span class="pl-c1">0</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; MPU TYPE: SEPARATE Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1048" class="blob-num js-line-number" data-line-number="1048"></td>
        <td id="LC1048" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU_TYPE_SEPARATE_Msk</span>              (<span class="pl-c1">1UL</span> &lt;&lt; MPU_TYPE_SEPARATE_Pos)                 <span class="pl-c"><span class="pl-c">/*</span>!&lt; MPU TYPE: SEPARATE Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1049" class="blob-num js-line-number" data-line-number="1049"></td>
        <td id="LC1049" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1050" class="blob-num js-line-number" data-line-number="1050"></td>
        <td id="LC1050" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> MPU Control Register <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1051" class="blob-num js-line-number" data-line-number="1051"></td>
        <td id="LC1051" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU_CTRL_PRIVDEFENA_Pos</span>             <span class="pl-c1">2</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; MPU CTRL: PRIVDEFENA Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1052" class="blob-num js-line-number" data-line-number="1052"></td>
        <td id="LC1052" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU_CTRL_PRIVDEFENA_Msk</span>            (<span class="pl-c1">1UL</span> &lt;&lt; MPU_CTRL_PRIVDEFENA_Pos)               <span class="pl-c"><span class="pl-c">/*</span>!&lt; MPU CTRL: PRIVDEFENA Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1053" class="blob-num js-line-number" data-line-number="1053"></td>
        <td id="LC1053" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1054" class="blob-num js-line-number" data-line-number="1054"></td>
        <td id="LC1054" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU_CTRL_HFNMIENA_Pos</span>               <span class="pl-c1">1</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; MPU CTRL: HFNMIENA Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1055" class="blob-num js-line-number" data-line-number="1055"></td>
        <td id="LC1055" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU_CTRL_HFNMIENA_Msk</span>              (<span class="pl-c1">1UL</span> &lt;&lt; MPU_CTRL_HFNMIENA_Pos)                 <span class="pl-c"><span class="pl-c">/*</span>!&lt; MPU CTRL: HFNMIENA Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1056" class="blob-num js-line-number" data-line-number="1056"></td>
        <td id="LC1056" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1057" class="blob-num js-line-number" data-line-number="1057"></td>
        <td id="LC1057" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU_CTRL_ENABLE_Pos</span>                 <span class="pl-c1">0</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; MPU CTRL: ENABLE Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1058" class="blob-num js-line-number" data-line-number="1058"></td>
        <td id="LC1058" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU_CTRL_ENABLE_Msk</span>                (<span class="pl-c1">1UL</span> &lt;&lt; MPU_CTRL_ENABLE_Pos)                   <span class="pl-c"><span class="pl-c">/*</span>!&lt; MPU CTRL: ENABLE Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1059" class="blob-num js-line-number" data-line-number="1059"></td>
        <td id="LC1059" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1060" class="blob-num js-line-number" data-line-number="1060"></td>
        <td id="LC1060" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> MPU Region Number Register <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1061" class="blob-num js-line-number" data-line-number="1061"></td>
        <td id="LC1061" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU_RNR_REGION_Pos</span>                  <span class="pl-c1">0</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; MPU RNR: REGION Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1062" class="blob-num js-line-number" data-line-number="1062"></td>
        <td id="LC1062" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU_RNR_REGION_Msk</span>                 (0xFFUL &lt;&lt; MPU_RNR_REGION_Pos)                 <span class="pl-c"><span class="pl-c">/*</span>!&lt; MPU RNR: REGION Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1063" class="blob-num js-line-number" data-line-number="1063"></td>
        <td id="LC1063" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1064" class="blob-num js-line-number" data-line-number="1064"></td>
        <td id="LC1064" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> MPU Region Base Address Register <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1065" class="blob-num js-line-number" data-line-number="1065"></td>
        <td id="LC1065" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU_RBAR_ADDR_Pos</span>                   <span class="pl-c1">5</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; MPU RBAR: ADDR Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1066" class="blob-num js-line-number" data-line-number="1066"></td>
        <td id="LC1066" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU_RBAR_ADDR_Msk</span>                  (0x7FFFFFFUL &lt;&lt; MPU_RBAR_ADDR_Pos)             <span class="pl-c"><span class="pl-c">/*</span>!&lt; MPU RBAR: ADDR Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1067" class="blob-num js-line-number" data-line-number="1067"></td>
        <td id="LC1067" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1068" class="blob-num js-line-number" data-line-number="1068"></td>
        <td id="LC1068" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU_RBAR_VALID_Pos</span>                  <span class="pl-c1">4</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; MPU RBAR: VALID Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1069" class="blob-num js-line-number" data-line-number="1069"></td>
        <td id="LC1069" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU_RBAR_VALID_Msk</span>                 (<span class="pl-c1">1UL</span> &lt;&lt; MPU_RBAR_VALID_Pos)                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; MPU RBAR: VALID Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1070" class="blob-num js-line-number" data-line-number="1070"></td>
        <td id="LC1070" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1071" class="blob-num js-line-number" data-line-number="1071"></td>
        <td id="LC1071" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU_RBAR_REGION_Pos</span>                 <span class="pl-c1">0</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; MPU RBAR: REGION Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1072" class="blob-num js-line-number" data-line-number="1072"></td>
        <td id="LC1072" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU_RBAR_REGION_Msk</span>                (0xFUL &lt;&lt; MPU_RBAR_REGION_Pos)                 <span class="pl-c"><span class="pl-c">/*</span>!&lt; MPU RBAR: REGION Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1073" class="blob-num js-line-number" data-line-number="1073"></td>
        <td id="LC1073" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1074" class="blob-num js-line-number" data-line-number="1074"></td>
        <td id="LC1074" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> MPU Region Attribute and Size Register <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1075" class="blob-num js-line-number" data-line-number="1075"></td>
        <td id="LC1075" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU_RASR_ATTRS_Pos</span>                 <span class="pl-c1">16</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; MPU RASR: MPU Region Attribute field Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1076" class="blob-num js-line-number" data-line-number="1076"></td>
        <td id="LC1076" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU_RASR_ATTRS_Msk</span>                 (0xFFFFUL &lt;&lt; MPU_RASR_ATTRS_Pos)               <span class="pl-c"><span class="pl-c">/*</span>!&lt; MPU RASR: MPU Region Attribute field Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1077" class="blob-num js-line-number" data-line-number="1077"></td>
        <td id="LC1077" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1078" class="blob-num js-line-number" data-line-number="1078"></td>
        <td id="LC1078" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU_RASR_XN_Pos</span>                    <span class="pl-c1">28</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; MPU RASR: ATTRS.XN Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1079" class="blob-num js-line-number" data-line-number="1079"></td>
        <td id="LC1079" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU_RASR_XN_Msk</span>                    (<span class="pl-c1">1UL</span> &lt;&lt; MPU_RASR_XN_Pos)                       <span class="pl-c"><span class="pl-c">/*</span>!&lt; MPU RASR: ATTRS.XN Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1080" class="blob-num js-line-number" data-line-number="1080"></td>
        <td id="LC1080" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1081" class="blob-num js-line-number" data-line-number="1081"></td>
        <td id="LC1081" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU_RASR_AP_Pos</span>                    <span class="pl-c1">24</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; MPU RASR: ATTRS.AP Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1082" class="blob-num js-line-number" data-line-number="1082"></td>
        <td id="LC1082" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU_RASR_AP_Msk</span>                    (0x7UL &lt;&lt; MPU_RASR_AP_Pos)                     <span class="pl-c"><span class="pl-c">/*</span>!&lt; MPU RASR: ATTRS.AP Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1083" class="blob-num js-line-number" data-line-number="1083"></td>
        <td id="LC1083" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1084" class="blob-num js-line-number" data-line-number="1084"></td>
        <td id="LC1084" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU_RASR_TEX_Pos</span>                   <span class="pl-c1">19</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; MPU RASR: ATTRS.TEX Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1085" class="blob-num js-line-number" data-line-number="1085"></td>
        <td id="LC1085" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU_RASR_TEX_Msk</span>                   (0x7UL &lt;&lt; MPU_RASR_TEX_Pos)                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; MPU RASR: ATTRS.TEX Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1086" class="blob-num js-line-number" data-line-number="1086"></td>
        <td id="LC1086" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1087" class="blob-num js-line-number" data-line-number="1087"></td>
        <td id="LC1087" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU_RASR_S_Pos</span>                     <span class="pl-c1">18</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; MPU RASR: ATTRS.S Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1088" class="blob-num js-line-number" data-line-number="1088"></td>
        <td id="LC1088" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU_RASR_S_Msk</span>                     (<span class="pl-c1">1UL</span> &lt;&lt; MPU_RASR_S_Pos)                        <span class="pl-c"><span class="pl-c">/*</span>!&lt; MPU RASR: ATTRS.S Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1089" class="blob-num js-line-number" data-line-number="1089"></td>
        <td id="LC1089" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1090" class="blob-num js-line-number" data-line-number="1090"></td>
        <td id="LC1090" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU_RASR_C_Pos</span>                     <span class="pl-c1">17</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; MPU RASR: ATTRS.C Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1091" class="blob-num js-line-number" data-line-number="1091"></td>
        <td id="LC1091" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU_RASR_C_Msk</span>                     (<span class="pl-c1">1UL</span> &lt;&lt; MPU_RASR_C_Pos)                        <span class="pl-c"><span class="pl-c">/*</span>!&lt; MPU RASR: ATTRS.C Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1092" class="blob-num js-line-number" data-line-number="1092"></td>
        <td id="LC1092" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1093" class="blob-num js-line-number" data-line-number="1093"></td>
        <td id="LC1093" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU_RASR_B_Pos</span>                     <span class="pl-c1">16</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; MPU RASR: ATTRS.B Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1094" class="blob-num js-line-number" data-line-number="1094"></td>
        <td id="LC1094" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU_RASR_B_Msk</span>                     (<span class="pl-c1">1UL</span> &lt;&lt; MPU_RASR_B_Pos)                        <span class="pl-c"><span class="pl-c">/*</span>!&lt; MPU RASR: ATTRS.B Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1095" class="blob-num js-line-number" data-line-number="1095"></td>
        <td id="LC1095" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1096" class="blob-num js-line-number" data-line-number="1096"></td>
        <td id="LC1096" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU_RASR_SRD_Pos</span>                    <span class="pl-c1">8</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; MPU RASR: Sub-Region Disable Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1097" class="blob-num js-line-number" data-line-number="1097"></td>
        <td id="LC1097" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU_RASR_SRD_Msk</span>                   (0xFFUL &lt;&lt; MPU_RASR_SRD_Pos)                   <span class="pl-c"><span class="pl-c">/*</span>!&lt; MPU RASR: Sub-Region Disable Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1098" class="blob-num js-line-number" data-line-number="1098"></td>
        <td id="LC1098" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1099" class="blob-num js-line-number" data-line-number="1099"></td>
        <td id="LC1099" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU_RASR_SIZE_Pos</span>                   <span class="pl-c1">1</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; MPU RASR: Region Size Field Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1100" class="blob-num js-line-number" data-line-number="1100"></td>
        <td id="LC1100" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU_RASR_SIZE_Msk</span>                  (0x1FUL &lt;&lt; MPU_RASR_SIZE_Pos)                  <span class="pl-c"><span class="pl-c">/*</span>!&lt; MPU RASR: Region Size Field Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1101" class="blob-num js-line-number" data-line-number="1101"></td>
        <td id="LC1101" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1102" class="blob-num js-line-number" data-line-number="1102"></td>
        <td id="LC1102" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU_RASR_ENABLE_Pos</span>                 <span class="pl-c1">0</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; MPU RASR: Region enable bit Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1103" class="blob-num js-line-number" data-line-number="1103"></td>
        <td id="LC1103" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU_RASR_ENABLE_Msk</span>                (<span class="pl-c1">1UL</span> &lt;&lt; MPU_RASR_ENABLE_Pos)                   <span class="pl-c"><span class="pl-c">/*</span>!&lt; MPU RASR: Region enable bit Disable Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1104" class="blob-num js-line-number" data-line-number="1104"></td>
        <td id="LC1104" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1105" class="blob-num js-line-number" data-line-number="1105"></td>
        <td id="LC1105" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>@} end of group CMSIS_MPU <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1106" class="blob-num js-line-number" data-line-number="1106"></td>
        <td id="LC1106" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">endif</span></td>
      </tr>
      <tr>
        <td id="L1107" class="blob-num js-line-number" data-line-number="1107"></td>
        <td id="LC1107" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1108" class="blob-num js-line-number" data-line-number="1108"></td>
        <td id="LC1108" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1109" class="blob-num js-line-number" data-line-number="1109"></td>
        <td id="LC1109" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \ingroup  CMSIS_core_register</span></td>
      </tr>
      <tr>
        <td id="L1110" class="blob-num js-line-number" data-line-number="1110"></td>
        <td id="LC1110" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \defgroup CMSIS_CoreDebug       Core Debug Registers (CoreDebug)</span></td>
      </tr>
      <tr>
        <td id="L1111" class="blob-num js-line-number" data-line-number="1111"></td>
        <td id="LC1111" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \brief      Type definitions for the Core Debug Registers</span></td>
      </tr>
      <tr>
        <td id="L1112" class="blob-num js-line-number" data-line-number="1112"></td>
        <td id="LC1112" class="blob-code blob-code-inner js-file-line"><span class="pl-c">  @{</span></td>
      </tr>
      <tr>
        <td id="L1113" class="blob-num js-line-number" data-line-number="1113"></td>
        <td id="LC1113" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1114" class="blob-num js-line-number" data-line-number="1114"></td>
        <td id="LC1114" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1115" class="blob-num js-line-number" data-line-number="1115"></td>
        <td id="LC1115" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \brief  Structure type to access the Core Debug Register (CoreDebug).</span></td>
      </tr>
      <tr>
        <td id="L1116" class="blob-num js-line-number" data-line-number="1116"></td>
        <td id="LC1116" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1117" class="blob-num js-line-number" data-line-number="1117"></td>
        <td id="LC1117" class="blob-code blob-code-inner js-file-line"><span class="pl-k">typedef</span> <span class="pl-k">struct</span></td>
      </tr>
      <tr>
        <td id="L1118" class="blob-num js-line-number" data-line-number="1118"></td>
        <td id="LC1118" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L1119" class="blob-num js-line-number" data-line-number="1119"></td>
        <td id="LC1119" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> DHCSR;                   <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x000 (R/W)  Debug Halting Control and Status Register    <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1120" class="blob-num js-line-number" data-line-number="1120"></td>
        <td id="LC1120" class="blob-code blob-code-inner js-file-line">  __O  <span class="pl-c1">uint32_t</span> DCRSR;                   <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x004 ( /W)  Debug Core Register Selector Register        <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1121" class="blob-num js-line-number" data-line-number="1121"></td>
        <td id="LC1121" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> DCRDR;                   <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x008 (R/W)  Debug Core Register Data Register            <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1122" class="blob-num js-line-number" data-line-number="1122"></td>
        <td id="LC1122" class="blob-code blob-code-inner js-file-line">  __IO <span class="pl-c1">uint32_t</span> DEMCR;                   <span class="pl-c"><span class="pl-c">/*</span>!&lt; Offset: 0x00C (R/W)  Debug Exception and Monitor Control Register <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1123" class="blob-num js-line-number" data-line-number="1123"></td>
        <td id="LC1123" class="blob-code blob-code-inner js-file-line">} CoreDebug_Type;</td>
      </tr>
      <tr>
        <td id="L1124" class="blob-num js-line-number" data-line-number="1124"></td>
        <td id="LC1124" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1125" class="blob-num js-line-number" data-line-number="1125"></td>
        <td id="LC1125" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> Debug Halting Control and Status Register <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1126" class="blob-num js-line-number" data-line-number="1126"></td>
        <td id="LC1126" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DHCSR_DBGKEY_Pos</span>         <span class="pl-c1">16</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DHCSR: DBGKEY Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1127" class="blob-num js-line-number" data-line-number="1127"></td>
        <td id="LC1127" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DHCSR_DBGKEY_Msk</span>         (0xFFFFUL &lt;&lt; CoreDebug_DHCSR_DBGKEY_Pos)       <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DHCSR: DBGKEY Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1128" class="blob-num js-line-number" data-line-number="1128"></td>
        <td id="LC1128" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1129" class="blob-num js-line-number" data-line-number="1129"></td>
        <td id="LC1129" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DHCSR_S_RESET_ST_Pos</span>     <span class="pl-c1">25</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DHCSR: S_RESET_ST Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1130" class="blob-num js-line-number" data-line-number="1130"></td>
        <td id="LC1130" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DHCSR_S_RESET_ST_Msk</span>     (<span class="pl-c1">1UL</span> &lt;&lt; CoreDebug_DHCSR_S_RESET_ST_Pos)        <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DHCSR: S_RESET_ST Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1131" class="blob-num js-line-number" data-line-number="1131"></td>
        <td id="LC1131" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1132" class="blob-num js-line-number" data-line-number="1132"></td>
        <td id="LC1132" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DHCSR_S_RETIRE_ST_Pos</span>    <span class="pl-c1">24</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DHCSR: S_RETIRE_ST Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1133" class="blob-num js-line-number" data-line-number="1133"></td>
        <td id="LC1133" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DHCSR_S_RETIRE_ST_Msk</span>    (<span class="pl-c1">1UL</span> &lt;&lt; CoreDebug_DHCSR_S_RETIRE_ST_Pos)       <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DHCSR: S_RETIRE_ST Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1134" class="blob-num js-line-number" data-line-number="1134"></td>
        <td id="LC1134" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1135" class="blob-num js-line-number" data-line-number="1135"></td>
        <td id="LC1135" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DHCSR_S_LOCKUP_Pos</span>       <span class="pl-c1">19</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DHCSR: S_LOCKUP Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1136" class="blob-num js-line-number" data-line-number="1136"></td>
        <td id="LC1136" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DHCSR_S_LOCKUP_Msk</span>       (<span class="pl-c1">1UL</span> &lt;&lt; CoreDebug_DHCSR_S_LOCKUP_Pos)          <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DHCSR: S_LOCKUP Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1137" class="blob-num js-line-number" data-line-number="1137"></td>
        <td id="LC1137" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1138" class="blob-num js-line-number" data-line-number="1138"></td>
        <td id="LC1138" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DHCSR_S_SLEEP_Pos</span>        <span class="pl-c1">18</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DHCSR: S_SLEEP Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1139" class="blob-num js-line-number" data-line-number="1139"></td>
        <td id="LC1139" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DHCSR_S_SLEEP_Msk</span>        (<span class="pl-c1">1UL</span> &lt;&lt; CoreDebug_DHCSR_S_SLEEP_Pos)           <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DHCSR: S_SLEEP Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1140" class="blob-num js-line-number" data-line-number="1140"></td>
        <td id="LC1140" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1141" class="blob-num js-line-number" data-line-number="1141"></td>
        <td id="LC1141" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DHCSR_S_HALT_Pos</span>         <span class="pl-c1">17</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DHCSR: S_HALT Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1142" class="blob-num js-line-number" data-line-number="1142"></td>
        <td id="LC1142" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DHCSR_S_HALT_Msk</span>         (<span class="pl-c1">1UL</span> &lt;&lt; CoreDebug_DHCSR_S_HALT_Pos)            <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DHCSR: S_HALT Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1143" class="blob-num js-line-number" data-line-number="1143"></td>
        <td id="LC1143" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1144" class="blob-num js-line-number" data-line-number="1144"></td>
        <td id="LC1144" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DHCSR_S_REGRDY_Pos</span>       <span class="pl-c1">16</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DHCSR: S_REGRDY Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1145" class="blob-num js-line-number" data-line-number="1145"></td>
        <td id="LC1145" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DHCSR_S_REGRDY_Msk</span>       (<span class="pl-c1">1UL</span> &lt;&lt; CoreDebug_DHCSR_S_REGRDY_Pos)          <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DHCSR: S_REGRDY Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1146" class="blob-num js-line-number" data-line-number="1146"></td>
        <td id="LC1146" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1147" class="blob-num js-line-number" data-line-number="1147"></td>
        <td id="LC1147" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DHCSR_C_SNAPSTALL_Pos</span>     <span class="pl-c1">5</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DHCSR: C_SNAPSTALL Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1148" class="blob-num js-line-number" data-line-number="1148"></td>
        <td id="LC1148" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DHCSR_C_SNAPSTALL_Msk</span>    (<span class="pl-c1">1UL</span> &lt;&lt; CoreDebug_DHCSR_C_SNAPSTALL_Pos)       <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DHCSR: C_SNAPSTALL Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1149" class="blob-num js-line-number" data-line-number="1149"></td>
        <td id="LC1149" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1150" class="blob-num js-line-number" data-line-number="1150"></td>
        <td id="LC1150" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DHCSR_C_MASKINTS_Pos</span>      <span class="pl-c1">3</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DHCSR: C_MASKINTS Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1151" class="blob-num js-line-number" data-line-number="1151"></td>
        <td id="LC1151" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DHCSR_C_MASKINTS_Msk</span>     (<span class="pl-c1">1UL</span> &lt;&lt; CoreDebug_DHCSR_C_MASKINTS_Pos)        <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DHCSR: C_MASKINTS Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1152" class="blob-num js-line-number" data-line-number="1152"></td>
        <td id="LC1152" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1153" class="blob-num js-line-number" data-line-number="1153"></td>
        <td id="LC1153" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DHCSR_C_STEP_Pos</span>          <span class="pl-c1">2</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DHCSR: C_STEP Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1154" class="blob-num js-line-number" data-line-number="1154"></td>
        <td id="LC1154" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DHCSR_C_STEP_Msk</span>         (<span class="pl-c1">1UL</span> &lt;&lt; CoreDebug_DHCSR_C_STEP_Pos)            <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DHCSR: C_STEP Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1155" class="blob-num js-line-number" data-line-number="1155"></td>
        <td id="LC1155" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1156" class="blob-num js-line-number" data-line-number="1156"></td>
        <td id="LC1156" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DHCSR_C_HALT_Pos</span>          <span class="pl-c1">1</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DHCSR: C_HALT Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1157" class="blob-num js-line-number" data-line-number="1157"></td>
        <td id="LC1157" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DHCSR_C_HALT_Msk</span>         (<span class="pl-c1">1UL</span> &lt;&lt; CoreDebug_DHCSR_C_HALT_Pos)            <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DHCSR: C_HALT Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1158" class="blob-num js-line-number" data-line-number="1158"></td>
        <td id="LC1158" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1159" class="blob-num js-line-number" data-line-number="1159"></td>
        <td id="LC1159" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DHCSR_C_DEBUGEN_Pos</span>       <span class="pl-c1">0</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DHCSR: C_DEBUGEN Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1160" class="blob-num js-line-number" data-line-number="1160"></td>
        <td id="LC1160" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DHCSR_C_DEBUGEN_Msk</span>      (<span class="pl-c1">1UL</span> &lt;&lt; CoreDebug_DHCSR_C_DEBUGEN_Pos)         <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DHCSR: C_DEBUGEN Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1161" class="blob-num js-line-number" data-line-number="1161"></td>
        <td id="LC1161" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1162" class="blob-num js-line-number" data-line-number="1162"></td>
        <td id="LC1162" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> Debug Core Register Selector Register <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1163" class="blob-num js-line-number" data-line-number="1163"></td>
        <td id="LC1163" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DCRSR_REGWnR_Pos</span>         <span class="pl-c1">16</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DCRSR: REGWnR Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1164" class="blob-num js-line-number" data-line-number="1164"></td>
        <td id="LC1164" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DCRSR_REGWnR_Msk</span>         (<span class="pl-c1">1UL</span> &lt;&lt; CoreDebug_DCRSR_REGWnR_Pos)            <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DCRSR: REGWnR Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1165" class="blob-num js-line-number" data-line-number="1165"></td>
        <td id="LC1165" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1166" class="blob-num js-line-number" data-line-number="1166"></td>
        <td id="LC1166" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DCRSR_REGSEL_Pos</span>          <span class="pl-c1">0</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DCRSR: REGSEL Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1167" class="blob-num js-line-number" data-line-number="1167"></td>
        <td id="LC1167" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DCRSR_REGSEL_Msk</span>         (0x1FUL &lt;&lt; CoreDebug_DCRSR_REGSEL_Pos)         <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DCRSR: REGSEL Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1168" class="blob-num js-line-number" data-line-number="1168"></td>
        <td id="LC1168" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1169" class="blob-num js-line-number" data-line-number="1169"></td>
        <td id="LC1169" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> Debug Exception and Monitor Control Register <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1170" class="blob-num js-line-number" data-line-number="1170"></td>
        <td id="LC1170" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DEMCR_TRCENA_Pos</span>         <span class="pl-c1">24</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DEMCR: TRCENA Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1171" class="blob-num js-line-number" data-line-number="1171"></td>
        <td id="LC1171" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DEMCR_TRCENA_Msk</span>         (<span class="pl-c1">1UL</span> &lt;&lt; CoreDebug_DEMCR_TRCENA_Pos)            <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DEMCR: TRCENA Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1172" class="blob-num js-line-number" data-line-number="1172"></td>
        <td id="LC1172" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1173" class="blob-num js-line-number" data-line-number="1173"></td>
        <td id="LC1173" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DEMCR_MON_REQ_Pos</span>        <span class="pl-c1">19</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DEMCR: MON_REQ Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1174" class="blob-num js-line-number" data-line-number="1174"></td>
        <td id="LC1174" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DEMCR_MON_REQ_Msk</span>        (<span class="pl-c1">1UL</span> &lt;&lt; CoreDebug_DEMCR_MON_REQ_Pos)           <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DEMCR: MON_REQ Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1175" class="blob-num js-line-number" data-line-number="1175"></td>
        <td id="LC1175" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1176" class="blob-num js-line-number" data-line-number="1176"></td>
        <td id="LC1176" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DEMCR_MON_STEP_Pos</span>       <span class="pl-c1">18</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DEMCR: MON_STEP Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1177" class="blob-num js-line-number" data-line-number="1177"></td>
        <td id="LC1177" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DEMCR_MON_STEP_Msk</span>       (<span class="pl-c1">1UL</span> &lt;&lt; CoreDebug_DEMCR_MON_STEP_Pos)          <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DEMCR: MON_STEP Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1178" class="blob-num js-line-number" data-line-number="1178"></td>
        <td id="LC1178" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1179" class="blob-num js-line-number" data-line-number="1179"></td>
        <td id="LC1179" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DEMCR_MON_PEND_Pos</span>       <span class="pl-c1">17</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DEMCR: MON_PEND Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1180" class="blob-num js-line-number" data-line-number="1180"></td>
        <td id="LC1180" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DEMCR_MON_PEND_Msk</span>       (<span class="pl-c1">1UL</span> &lt;&lt; CoreDebug_DEMCR_MON_PEND_Pos)          <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DEMCR: MON_PEND Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1181" class="blob-num js-line-number" data-line-number="1181"></td>
        <td id="LC1181" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1182" class="blob-num js-line-number" data-line-number="1182"></td>
        <td id="LC1182" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DEMCR_MON_EN_Pos</span>         <span class="pl-c1">16</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DEMCR: MON_EN Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1183" class="blob-num js-line-number" data-line-number="1183"></td>
        <td id="LC1183" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DEMCR_MON_EN_Msk</span>         (<span class="pl-c1">1UL</span> &lt;&lt; CoreDebug_DEMCR_MON_EN_Pos)            <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DEMCR: MON_EN Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1184" class="blob-num js-line-number" data-line-number="1184"></td>
        <td id="LC1184" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1185" class="blob-num js-line-number" data-line-number="1185"></td>
        <td id="LC1185" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DEMCR_VC_HARDERR_Pos</span>     <span class="pl-c1">10</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DEMCR: VC_HARDERR Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1186" class="blob-num js-line-number" data-line-number="1186"></td>
        <td id="LC1186" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DEMCR_VC_HARDERR_Msk</span>     (<span class="pl-c1">1UL</span> &lt;&lt; CoreDebug_DEMCR_VC_HARDERR_Pos)        <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DEMCR: VC_HARDERR Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1187" class="blob-num js-line-number" data-line-number="1187"></td>
        <td id="LC1187" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1188" class="blob-num js-line-number" data-line-number="1188"></td>
        <td id="LC1188" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DEMCR_VC_INTERR_Pos</span>       <span class="pl-c1">9</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DEMCR: VC_INTERR Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1189" class="blob-num js-line-number" data-line-number="1189"></td>
        <td id="LC1189" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DEMCR_VC_INTERR_Msk</span>      (<span class="pl-c1">1UL</span> &lt;&lt; CoreDebug_DEMCR_VC_INTERR_Pos)         <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DEMCR: VC_INTERR Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1190" class="blob-num js-line-number" data-line-number="1190"></td>
        <td id="LC1190" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1191" class="blob-num js-line-number" data-line-number="1191"></td>
        <td id="LC1191" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DEMCR_VC_BUSERR_Pos</span>       <span class="pl-c1">8</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DEMCR: VC_BUSERR Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1192" class="blob-num js-line-number" data-line-number="1192"></td>
        <td id="LC1192" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DEMCR_VC_BUSERR_Msk</span>      (<span class="pl-c1">1UL</span> &lt;&lt; CoreDebug_DEMCR_VC_BUSERR_Pos)         <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DEMCR: VC_BUSERR Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1193" class="blob-num js-line-number" data-line-number="1193"></td>
        <td id="LC1193" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1194" class="blob-num js-line-number" data-line-number="1194"></td>
        <td id="LC1194" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DEMCR_VC_STATERR_Pos</span>      <span class="pl-c1">7</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DEMCR: VC_STATERR Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1195" class="blob-num js-line-number" data-line-number="1195"></td>
        <td id="LC1195" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DEMCR_VC_STATERR_Msk</span>     (<span class="pl-c1">1UL</span> &lt;&lt; CoreDebug_DEMCR_VC_STATERR_Pos)        <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DEMCR: VC_STATERR Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1196" class="blob-num js-line-number" data-line-number="1196"></td>
        <td id="LC1196" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1197" class="blob-num js-line-number" data-line-number="1197"></td>
        <td id="LC1197" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DEMCR_VC_CHKERR_Pos</span>       <span class="pl-c1">6</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DEMCR: VC_CHKERR Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1198" class="blob-num js-line-number" data-line-number="1198"></td>
        <td id="LC1198" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DEMCR_VC_CHKERR_Msk</span>      (<span class="pl-c1">1UL</span> &lt;&lt; CoreDebug_DEMCR_VC_CHKERR_Pos)         <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DEMCR: VC_CHKERR Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1199" class="blob-num js-line-number" data-line-number="1199"></td>
        <td id="LC1199" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1200" class="blob-num js-line-number" data-line-number="1200"></td>
        <td id="LC1200" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DEMCR_VC_NOCPERR_Pos</span>      <span class="pl-c1">5</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DEMCR: VC_NOCPERR Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1201" class="blob-num js-line-number" data-line-number="1201"></td>
        <td id="LC1201" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DEMCR_VC_NOCPERR_Msk</span>     (<span class="pl-c1">1UL</span> &lt;&lt; CoreDebug_DEMCR_VC_NOCPERR_Pos)        <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DEMCR: VC_NOCPERR Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1202" class="blob-num js-line-number" data-line-number="1202"></td>
        <td id="LC1202" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1203" class="blob-num js-line-number" data-line-number="1203"></td>
        <td id="LC1203" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DEMCR_VC_MMERR_Pos</span>        <span class="pl-c1">4</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DEMCR: VC_MMERR Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1204" class="blob-num js-line-number" data-line-number="1204"></td>
        <td id="LC1204" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DEMCR_VC_MMERR_Msk</span>       (<span class="pl-c1">1UL</span> &lt;&lt; CoreDebug_DEMCR_VC_MMERR_Pos)          <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DEMCR: VC_MMERR Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1205" class="blob-num js-line-number" data-line-number="1205"></td>
        <td id="LC1205" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1206" class="blob-num js-line-number" data-line-number="1206"></td>
        <td id="LC1206" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DEMCR_VC_CORERESET_Pos</span>    <span class="pl-c1">0</span>                                             <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DEMCR: VC_CORERESET Position <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1207" class="blob-num js-line-number" data-line-number="1207"></td>
        <td id="LC1207" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_DEMCR_VC_CORERESET_Msk</span>   (<span class="pl-c1">1UL</span> &lt;&lt; CoreDebug_DEMCR_VC_CORERESET_Pos)      <span class="pl-c"><span class="pl-c">/*</span>!&lt; CoreDebug DEMCR: VC_CORERESET Mask <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1208" class="blob-num js-line-number" data-line-number="1208"></td>
        <td id="LC1208" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1209" class="blob-num js-line-number" data-line-number="1209"></td>
        <td id="LC1209" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>@} end of group CMSIS_CoreDebug <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1210" class="blob-num js-line-number" data-line-number="1210"></td>
        <td id="LC1210" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1211" class="blob-num js-line-number" data-line-number="1211"></td>
        <td id="LC1211" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1212" class="blob-num js-line-number" data-line-number="1212"></td>
        <td id="LC1212" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \ingroup    CMSIS_core_register</span></td>
      </tr>
      <tr>
        <td id="L1213" class="blob-num js-line-number" data-line-number="1213"></td>
        <td id="LC1213" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \defgroup   CMSIS_core_base     Core Definitions</span></td>
      </tr>
      <tr>
        <td id="L1214" class="blob-num js-line-number" data-line-number="1214"></td>
        <td id="LC1214" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \brief      Definitions for base addresses, unions, and structures.</span></td>
      </tr>
      <tr>
        <td id="L1215" class="blob-num js-line-number" data-line-number="1215"></td>
        <td id="LC1215" class="blob-code blob-code-inner js-file-line"><span class="pl-c">  @{</span></td>
      </tr>
      <tr>
        <td id="L1216" class="blob-num js-line-number" data-line-number="1216"></td>
        <td id="LC1216" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1217" class="blob-num js-line-number" data-line-number="1217"></td>
        <td id="LC1217" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1218" class="blob-num js-line-number" data-line-number="1218"></td>
        <td id="LC1218" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> Memory mapping of Cortex-M3 Hardware <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1219" class="blob-num js-line-number" data-line-number="1219"></td>
        <td id="LC1219" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCS_BASE</span>            (0xE000E000UL)                            <span class="pl-c"><span class="pl-c">/*</span>!&lt; System Control Space Base Address  <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1220" class="blob-num js-line-number" data-line-number="1220"></td>
        <td id="LC1220" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">ITM_BASE</span>            (0xE0000000UL)                            <span class="pl-c"><span class="pl-c">/*</span>!&lt; ITM Base Address                   <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1221" class="blob-num js-line-number" data-line-number="1221"></td>
        <td id="LC1221" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT_BASE</span>            (0xE0001000UL)                            <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT Base Address                   <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1222" class="blob-num js-line-number" data-line-number="1222"></td>
        <td id="LC1222" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI_BASE</span>            (0xE0040000UL)                            <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI Base Address                   <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1223" class="blob-num js-line-number" data-line-number="1223"></td>
        <td id="LC1223" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug_BASE</span>      (0xE000EDF0UL)                            <span class="pl-c"><span class="pl-c">/*</span>!&lt; Core Debug Base Address            <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1224" class="blob-num js-line-number" data-line-number="1224"></td>
        <td id="LC1224" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SysTick_BASE</span>        (SCS_BASE +  0x0010UL)                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; SysTick Base Address               <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1225" class="blob-num js-line-number" data-line-number="1225"></td>
        <td id="LC1225" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">NVIC_BASE</span>           (SCS_BASE +  0x0100UL)                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; NVIC Base Address                  <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1226" class="blob-num js-line-number" data-line-number="1226"></td>
        <td id="LC1226" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB_BASE</span>            (SCS_BASE +  0x0D00UL)                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; System Control Block Base Address  <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1227" class="blob-num js-line-number" data-line-number="1227"></td>
        <td id="LC1227" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1228" class="blob-num js-line-number" data-line-number="1228"></td>
        <td id="LC1228" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCnSCB</span>              ((SCnSCB_Type    *)     SCS_BASE      )   <span class="pl-c"><span class="pl-c">/*</span>!&lt; System control Register not in SCB <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1229" class="blob-num js-line-number" data-line-number="1229"></td>
        <td id="LC1229" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SCB</span>                 ((SCB_Type       *)     SCB_BASE      )   <span class="pl-c"><span class="pl-c">/*</span>!&lt; SCB configuration struct           <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1230" class="blob-num js-line-number" data-line-number="1230"></td>
        <td id="LC1230" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">SysTick</span>             ((SysTick_Type   *)     SysTick_BASE  )   <span class="pl-c"><span class="pl-c">/*</span>!&lt; SysTick configuration struct       <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1231" class="blob-num js-line-number" data-line-number="1231"></td>
        <td id="LC1231" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">NVIC</span>                ((NVIC_Type      *)     NVIC_BASE     )   <span class="pl-c"><span class="pl-c">/*</span>!&lt; NVIC configuration struct          <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1232" class="blob-num js-line-number" data-line-number="1232"></td>
        <td id="LC1232" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">ITM</span>                 ((ITM_Type       *)     ITM_BASE      )   <span class="pl-c"><span class="pl-c">/*</span>!&lt; ITM configuration struct           <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1233" class="blob-num js-line-number" data-line-number="1233"></td>
        <td id="LC1233" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DWT</span>                 ((DWT_Type       *)     DWT_BASE      )   <span class="pl-c"><span class="pl-c">/*</span>!&lt; DWT configuration struct           <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1234" class="blob-num js-line-number" data-line-number="1234"></td>
        <td id="LC1234" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">TPI</span>                 ((TPI_Type       *)     TPI_BASE      )   <span class="pl-c"><span class="pl-c">/*</span>!&lt; TPI configuration struct           <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1235" class="blob-num js-line-number" data-line-number="1235"></td>
        <td id="LC1235" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">CoreDebug</span>           ((CoreDebug_Type *)     CoreDebug_BASE)   <span class="pl-c"><span class="pl-c">/*</span>!&lt; Core Debug configuration struct    <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1236" class="blob-num js-line-number" data-line-number="1236"></td>
        <td id="LC1236" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1237" class="blob-num js-line-number" data-line-number="1237"></td>
        <td id="LC1237" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">if</span> (__MPU_PRESENT == 1)</td>
      </tr>
      <tr>
        <td id="L1238" class="blob-num js-line-number" data-line-number="1238"></td>
        <td id="LC1238" class="blob-code blob-code-inner js-file-line">  #<span class="pl-k">define</span> <span class="pl-en">MPU_BASE</span>          (SCS_BASE +  0x0D90UL)                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; Memory Protection Unit             <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1239" class="blob-num js-line-number" data-line-number="1239"></td>
        <td id="LC1239" class="blob-code blob-code-inner js-file-line">  #<span class="pl-k">define</span> <span class="pl-en">MPU</span>               ((MPU_Type       *)     MPU_BASE      )   <span class="pl-c"><span class="pl-c">/*</span>!&lt; Memory Protection Unit             <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1240" class="blob-num js-line-number" data-line-number="1240"></td>
        <td id="LC1240" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">endif</span></td>
      </tr>
      <tr>
        <td id="L1241" class="blob-num js-line-number" data-line-number="1241"></td>
        <td id="LC1241" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1242" class="blob-num js-line-number" data-line-number="1242"></td>
        <td id="LC1242" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>@} <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1243" class="blob-num js-line-number" data-line-number="1243"></td>
        <td id="LC1243" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1244" class="blob-num js-line-number" data-line-number="1244"></td>
        <td id="LC1244" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1245" class="blob-num js-line-number" data-line-number="1245"></td>
        <td id="LC1245" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1246" class="blob-num js-line-number" data-line-number="1246"></td>
        <td id="LC1246" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>******************************************************************************</span></td>
      </tr>
      <tr>
        <td id="L1247" class="blob-num js-line-number" data-line-number="1247"></td>
        <td id="LC1247" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> *                Hardware Abstraction Layer</span></td>
      </tr>
      <tr>
        <td id="L1248" class="blob-num js-line-number" data-line-number="1248"></td>
        <td id="LC1248" class="blob-code blob-code-inner js-file-line"><span class="pl-c">  Core Function Interface contains:</span></td>
      </tr>
      <tr>
        <td id="L1249" class="blob-num js-line-number" data-line-number="1249"></td>
        <td id="LC1249" class="blob-code blob-code-inner js-file-line"><span class="pl-c">  - Core NVIC Functions</span></td>
      </tr>
      <tr>
        <td id="L1250" class="blob-num js-line-number" data-line-number="1250"></td>
        <td id="LC1250" class="blob-code blob-code-inner js-file-line"><span class="pl-c">  - Core SysTick Functions</span></td>
      </tr>
      <tr>
        <td id="L1251" class="blob-num js-line-number" data-line-number="1251"></td>
        <td id="LC1251" class="blob-code blob-code-inner js-file-line"><span class="pl-c">  - Core Debug Functions</span></td>
      </tr>
      <tr>
        <td id="L1252" class="blob-num js-line-number" data-line-number="1252"></td>
        <td id="LC1252" class="blob-code blob-code-inner js-file-line"><span class="pl-c">  - Core Register Access Functions</span></td>
      </tr>
      <tr>
        <td id="L1253" class="blob-num js-line-number" data-line-number="1253"></td>
        <td id="LC1253" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> *****************************************************************************<span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1254" class="blob-num js-line-number" data-line-number="1254"></td>
        <td id="LC1254" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \defgroup CMSIS_Core_FunctionInterface Functions and Instructions Reference</span></td>
      </tr>
      <tr>
        <td id="L1255" class="blob-num js-line-number" data-line-number="1255"></td>
        <td id="LC1255" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1256" class="blob-num js-line-number" data-line-number="1256"></td>
        <td id="LC1256" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1257" class="blob-num js-line-number" data-line-number="1257"></td>
        <td id="LC1257" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1258" class="blob-num js-line-number" data-line-number="1258"></td>
        <td id="LC1258" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1259" class="blob-num js-line-number" data-line-number="1259"></td>
        <td id="LC1259" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> ##########################   NVIC functions  #################################### <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1260" class="blob-num js-line-number" data-line-number="1260"></td>
        <td id="LC1260" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \ingroup  CMSIS_Core_FunctionInterface</span></td>
      </tr>
      <tr>
        <td id="L1261" class="blob-num js-line-number" data-line-number="1261"></td>
        <td id="LC1261" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \defgroup CMSIS_Core_NVICFunctions NVIC Functions</span></td>
      </tr>
      <tr>
        <td id="L1262" class="blob-num js-line-number" data-line-number="1262"></td>
        <td id="LC1262" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \brief      Functions that manage interrupts and exceptions via the NVIC.</span></td>
      </tr>
      <tr>
        <td id="L1263" class="blob-num js-line-number" data-line-number="1263"></td>
        <td id="LC1263" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    @{</span></td>
      </tr>
      <tr>
        <td id="L1264" class="blob-num js-line-number" data-line-number="1264"></td>
        <td id="LC1264" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1265" class="blob-num js-line-number" data-line-number="1265"></td>
        <td id="LC1265" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1266" class="blob-num js-line-number" data-line-number="1266"></td>
        <td id="LC1266" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \brief  Set Priority Grouping</span></td>
      </tr>
      <tr>
        <td id="L1267" class="blob-num js-line-number" data-line-number="1267"></td>
        <td id="LC1267" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L1268" class="blob-num js-line-number" data-line-number="1268"></td>
        <td id="LC1268" class="blob-code blob-code-inner js-file-line"><span class="pl-c">  The function sets the priority grouping field using the required unlock sequence.</span></td>
      </tr>
      <tr>
        <td id="L1269" class="blob-num js-line-number" data-line-number="1269"></td>
        <td id="LC1269" class="blob-code blob-code-inner js-file-line"><span class="pl-c">  The parameter PriorityGroup is assigned to the field SCB-&gt;AIRCR [10:8] PRIGROUP field.</span></td>
      </tr>
      <tr>
        <td id="L1270" class="blob-num js-line-number" data-line-number="1270"></td>
        <td id="LC1270" class="blob-code blob-code-inner js-file-line"><span class="pl-c">  Only values from 0..7 are used.</span></td>
      </tr>
      <tr>
        <td id="L1271" class="blob-num js-line-number" data-line-number="1271"></td>
        <td id="LC1271" class="blob-code blob-code-inner js-file-line"><span class="pl-c">  In case of a conflict between priority grouping and available</span></td>
      </tr>
      <tr>
        <td id="L1272" class="blob-num js-line-number" data-line-number="1272"></td>
        <td id="LC1272" class="blob-code blob-code-inner js-file-line"><span class="pl-c">  priority bits (__NVIC_PRIO_BITS), the smallest possible priority group is set.</span></td>
      </tr>
      <tr>
        <td id="L1273" class="blob-num js-line-number" data-line-number="1273"></td>
        <td id="LC1273" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L1274" class="blob-num js-line-number" data-line-number="1274"></td>
        <td id="LC1274" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \param [in]      PriorityGroup  Priority grouping field.</span></td>
      </tr>
      <tr>
        <td id="L1275" class="blob-num js-line-number" data-line-number="1275"></td>
        <td id="LC1275" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1276" class="blob-num js-line-number" data-line-number="1276"></td>
        <td id="LC1276" class="blob-code blob-code-inner js-file-line">__STATIC_INLINE <span class="pl-k">void</span> <span class="pl-smi">NVIC_SetPriorityGrouping</span>(<span class="pl-c1">uint32_t</span> PriorityGroup)</td>
      </tr>
      <tr>
        <td id="L1277" class="blob-num js-line-number" data-line-number="1277"></td>
        <td id="LC1277" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L1278" class="blob-num js-line-number" data-line-number="1278"></td>
        <td id="LC1278" class="blob-code blob-code-inner js-file-line">  <span class="pl-c1">uint32_t</span> reg_value;</td>
      </tr>
      <tr>
        <td id="L1279" class="blob-num js-line-number" data-line-number="1279"></td>
        <td id="LC1279" class="blob-code blob-code-inner js-file-line">  <span class="pl-c1">uint32_t</span> PriorityGroupTmp = (PriorityGroup &amp; (<span class="pl-c1">uint32_t</span>)0x07);               <span class="pl-c"><span class="pl-c">/*</span> only values 0..7 are used          <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1280" class="blob-num js-line-number" data-line-number="1280"></td>
        <td id="LC1280" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1281" class="blob-num js-line-number" data-line-number="1281"></td>
        <td id="LC1281" class="blob-code blob-code-inner js-file-line">  reg_value  =  SCB-&gt;AIRCR;                                                   <span class="pl-c"><span class="pl-c">/*</span> read old register configuration    <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1282" class="blob-num js-line-number" data-line-number="1282"></td>
        <td id="LC1282" class="blob-code blob-code-inner js-file-line">  reg_value &amp;= ~(SCB_AIRCR_VECTKEY_Msk | SCB_AIRCR_PRIGROUP_Msk);             <span class="pl-c"><span class="pl-c">/*</span> clear bits to change               <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1283" class="blob-num js-line-number" data-line-number="1283"></td>
        <td id="LC1283" class="blob-code blob-code-inner js-file-line">  reg_value  =  (reg_value                                 |</td>
      </tr>
      <tr>
        <td id="L1284" class="blob-num js-line-number" data-line-number="1284"></td>
        <td id="LC1284" class="blob-code blob-code-inner js-file-line">                ((<span class="pl-c1">uint32_t</span>)0x5FA &lt;&lt; SCB_AIRCR_VECTKEY_Pos) |</td>
      </tr>
      <tr>
        <td id="L1285" class="blob-num js-line-number" data-line-number="1285"></td>
        <td id="LC1285" class="blob-code blob-code-inner js-file-line">                (PriorityGroupTmp &lt;&lt; <span class="pl-c1">8</span>));                                     <span class="pl-c"><span class="pl-c">/*</span> Insert write key and priorty group <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1286" class="blob-num js-line-number" data-line-number="1286"></td>
        <td id="LC1286" class="blob-code blob-code-inner js-file-line">  SCB-&gt;AIRCR =  reg_value;</td>
      </tr>
      <tr>
        <td id="L1287" class="blob-num js-line-number" data-line-number="1287"></td>
        <td id="LC1287" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L1288" class="blob-num js-line-number" data-line-number="1288"></td>
        <td id="LC1288" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1289" class="blob-num js-line-number" data-line-number="1289"></td>
        <td id="LC1289" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1290" class="blob-num js-line-number" data-line-number="1290"></td>
        <td id="LC1290" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \brief  Get Priority Grouping</span></td>
      </tr>
      <tr>
        <td id="L1291" class="blob-num js-line-number" data-line-number="1291"></td>
        <td id="LC1291" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L1292" class="blob-num js-line-number" data-line-number="1292"></td>
        <td id="LC1292" class="blob-code blob-code-inner js-file-line"><span class="pl-c">  The function reads the priority grouping field from the NVIC Interrupt Controller.</span></td>
      </tr>
      <tr>
        <td id="L1293" class="blob-num js-line-number" data-line-number="1293"></td>
        <td id="LC1293" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L1294" class="blob-num js-line-number" data-line-number="1294"></td>
        <td id="LC1294" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \return                Priority grouping field (SCB-&gt;AIRCR [10:8] PRIGROUP field).</span></td>
      </tr>
      <tr>
        <td id="L1295" class="blob-num js-line-number" data-line-number="1295"></td>
        <td id="LC1295" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1296" class="blob-num js-line-number" data-line-number="1296"></td>
        <td id="LC1296" class="blob-code blob-code-inner js-file-line">__STATIC_INLINE <span class="pl-c1">uint32_t</span> <span class="pl-smi">NVIC_GetPriorityGrouping</span>(<span class="pl-k">void</span>)</td>
      </tr>
      <tr>
        <td id="L1297" class="blob-num js-line-number" data-line-number="1297"></td>
        <td id="LC1297" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L1298" class="blob-num js-line-number" data-line-number="1298"></td>
        <td id="LC1298" class="blob-code blob-code-inner js-file-line">  <span class="pl-k">return</span> ((SCB-&gt;AIRCR &amp; SCB_AIRCR_PRIGROUP_Msk) &gt;&gt; SCB_AIRCR_PRIGROUP_Pos);   <span class="pl-c"><span class="pl-c">/*</span> read priority grouping field <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1299" class="blob-num js-line-number" data-line-number="1299"></td>
        <td id="LC1299" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L1300" class="blob-num js-line-number" data-line-number="1300"></td>
        <td id="LC1300" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1301" class="blob-num js-line-number" data-line-number="1301"></td>
        <td id="LC1301" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1302" class="blob-num js-line-number" data-line-number="1302"></td>
        <td id="LC1302" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \brief  Enable External Interrupt</span></td>
      </tr>
      <tr>
        <td id="L1303" class="blob-num js-line-number" data-line-number="1303"></td>
        <td id="LC1303" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L1304" class="blob-num js-line-number" data-line-number="1304"></td>
        <td id="LC1304" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    The function enables a device-specific interrupt in the NVIC interrupt controller.</span></td>
      </tr>
      <tr>
        <td id="L1305" class="blob-num js-line-number" data-line-number="1305"></td>
        <td id="LC1305" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L1306" class="blob-num js-line-number" data-line-number="1306"></td>
        <td id="LC1306" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \param [in]      IRQn  External interrupt number. Value cannot be negative.</span></td>
      </tr>
      <tr>
        <td id="L1307" class="blob-num js-line-number" data-line-number="1307"></td>
        <td id="LC1307" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1308" class="blob-num js-line-number" data-line-number="1308"></td>
        <td id="LC1308" class="blob-code blob-code-inner js-file-line">__STATIC_INLINE <span class="pl-k">void</span> <span class="pl-smi">NVIC_EnableIRQ</span>(IRQn_Type IRQn)</td>
      </tr>
      <tr>
        <td id="L1309" class="blob-num js-line-number" data-line-number="1309"></td>
        <td id="LC1309" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L1310" class="blob-num js-line-number" data-line-number="1310"></td>
        <td id="LC1310" class="blob-code blob-code-inner js-file-line">  NVIC-&gt;ISER[((<span class="pl-c1">uint32_t</span>)(IRQn) &gt;&gt; <span class="pl-c1">5</span>)] = (<span class="pl-c1">1</span> &lt;&lt; ((<span class="pl-c1">uint32_t</span>)(IRQn) &amp; 0x1F)); <span class="pl-c"><span class="pl-c">/*</span> enable interrupt <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1311" class="blob-num js-line-number" data-line-number="1311"></td>
        <td id="LC1311" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L1312" class="blob-num js-line-number" data-line-number="1312"></td>
        <td id="LC1312" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1313" class="blob-num js-line-number" data-line-number="1313"></td>
        <td id="LC1313" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1314" class="blob-num js-line-number" data-line-number="1314"></td>
        <td id="LC1314" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \brief  Disable External Interrupt</span></td>
      </tr>
      <tr>
        <td id="L1315" class="blob-num js-line-number" data-line-number="1315"></td>
        <td id="LC1315" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L1316" class="blob-num js-line-number" data-line-number="1316"></td>
        <td id="LC1316" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    The function disables a device-specific interrupt in the NVIC interrupt controller.</span></td>
      </tr>
      <tr>
        <td id="L1317" class="blob-num js-line-number" data-line-number="1317"></td>
        <td id="LC1317" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L1318" class="blob-num js-line-number" data-line-number="1318"></td>
        <td id="LC1318" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \param [in]      IRQn  External interrupt number. Value cannot be negative.</span></td>
      </tr>
      <tr>
        <td id="L1319" class="blob-num js-line-number" data-line-number="1319"></td>
        <td id="LC1319" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1320" class="blob-num js-line-number" data-line-number="1320"></td>
        <td id="LC1320" class="blob-code blob-code-inner js-file-line">__STATIC_INLINE <span class="pl-k">void</span> <span class="pl-smi">NVIC_DisableIRQ</span>(IRQn_Type IRQn)</td>
      </tr>
      <tr>
        <td id="L1321" class="blob-num js-line-number" data-line-number="1321"></td>
        <td id="LC1321" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L1322" class="blob-num js-line-number" data-line-number="1322"></td>
        <td id="LC1322" class="blob-code blob-code-inner js-file-line">  NVIC-&gt;ICER[((<span class="pl-c1">uint32_t</span>)(IRQn) &gt;&gt; <span class="pl-c1">5</span>)] = (<span class="pl-c1">1</span> &lt;&lt; ((<span class="pl-c1">uint32_t</span>)(IRQn) &amp; 0x1F)); <span class="pl-c"><span class="pl-c">/*</span> disable interrupt <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1323" class="blob-num js-line-number" data-line-number="1323"></td>
        <td id="LC1323" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L1324" class="blob-num js-line-number" data-line-number="1324"></td>
        <td id="LC1324" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1325" class="blob-num js-line-number" data-line-number="1325"></td>
        <td id="LC1325" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1326" class="blob-num js-line-number" data-line-number="1326"></td>
        <td id="LC1326" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \brief  Get Pending Interrupt</span></td>
      </tr>
      <tr>
        <td id="L1327" class="blob-num js-line-number" data-line-number="1327"></td>
        <td id="LC1327" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L1328" class="blob-num js-line-number" data-line-number="1328"></td>
        <td id="LC1328" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    The function reads the pending register in the NVIC and returns the pending bit</span></td>
      </tr>
      <tr>
        <td id="L1329" class="blob-num js-line-number" data-line-number="1329"></td>
        <td id="LC1329" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    for the specified interrupt.</span></td>
      </tr>
      <tr>
        <td id="L1330" class="blob-num js-line-number" data-line-number="1330"></td>
        <td id="LC1330" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L1331" class="blob-num js-line-number" data-line-number="1331"></td>
        <td id="LC1331" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \param [in]      IRQn  Interrupt number.</span></td>
      </tr>
      <tr>
        <td id="L1332" class="blob-num js-line-number" data-line-number="1332"></td>
        <td id="LC1332" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L1333" class="blob-num js-line-number" data-line-number="1333"></td>
        <td id="LC1333" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \return             0  Interrupt status is not pending.</span></td>
      </tr>
      <tr>
        <td id="L1334" class="blob-num js-line-number" data-line-number="1334"></td>
        <td id="LC1334" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \return             1  Interrupt status is pending.</span></td>
      </tr>
      <tr>
        <td id="L1335" class="blob-num js-line-number" data-line-number="1335"></td>
        <td id="LC1335" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1336" class="blob-num js-line-number" data-line-number="1336"></td>
        <td id="LC1336" class="blob-code blob-code-inner js-file-line">__STATIC_INLINE <span class="pl-c1">uint32_t</span> <span class="pl-smi">NVIC_GetPendingIRQ</span>(IRQn_Type IRQn)</td>
      </tr>
      <tr>
        <td id="L1337" class="blob-num js-line-number" data-line-number="1337"></td>
        <td id="LC1337" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L1338" class="blob-num js-line-number" data-line-number="1338"></td>
        <td id="LC1338" class="blob-code blob-code-inner js-file-line">  <span class="pl-k">return</span>((<span class="pl-c1">uint32_t</span>) ((NVIC-&gt;ISPR[(<span class="pl-c1">uint32_t</span>)(IRQn) &gt;&gt; <span class="pl-c1">5</span>] &amp; (<span class="pl-c1">1</span> &lt;&lt; ((<span class="pl-c1">uint32_t</span>)(IRQn) &amp; 0x1F)))?<span class="pl-c1">1</span>:<span class="pl-c1">0</span>)); <span class="pl-c"><span class="pl-c">/*</span> Return 1 if pending else 0 <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1339" class="blob-num js-line-number" data-line-number="1339"></td>
        <td id="LC1339" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L1340" class="blob-num js-line-number" data-line-number="1340"></td>
        <td id="LC1340" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1341" class="blob-num js-line-number" data-line-number="1341"></td>
        <td id="LC1341" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1342" class="blob-num js-line-number" data-line-number="1342"></td>
        <td id="LC1342" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \brief  Set Pending Interrupt</span></td>
      </tr>
      <tr>
        <td id="L1343" class="blob-num js-line-number" data-line-number="1343"></td>
        <td id="LC1343" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L1344" class="blob-num js-line-number" data-line-number="1344"></td>
        <td id="LC1344" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    The function sets the pending bit of an external interrupt.</span></td>
      </tr>
      <tr>
        <td id="L1345" class="blob-num js-line-number" data-line-number="1345"></td>
        <td id="LC1345" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L1346" class="blob-num js-line-number" data-line-number="1346"></td>
        <td id="LC1346" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \param [in]      IRQn  Interrupt number. Value cannot be negative.</span></td>
      </tr>
      <tr>
        <td id="L1347" class="blob-num js-line-number" data-line-number="1347"></td>
        <td id="LC1347" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1348" class="blob-num js-line-number" data-line-number="1348"></td>
        <td id="LC1348" class="blob-code blob-code-inner js-file-line">__STATIC_INLINE <span class="pl-k">void</span> <span class="pl-smi">NVIC_SetPendingIRQ</span>(IRQn_Type IRQn)</td>
      </tr>
      <tr>
        <td id="L1349" class="blob-num js-line-number" data-line-number="1349"></td>
        <td id="LC1349" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L1350" class="blob-num js-line-number" data-line-number="1350"></td>
        <td id="LC1350" class="blob-code blob-code-inner js-file-line">  NVIC-&gt;ISPR[((<span class="pl-c1">uint32_t</span>)(IRQn) &gt;&gt; <span class="pl-c1">5</span>)] = (<span class="pl-c1">1</span> &lt;&lt; ((<span class="pl-c1">uint32_t</span>)(IRQn) &amp; 0x1F)); <span class="pl-c"><span class="pl-c">/*</span> set interrupt pending <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1351" class="blob-num js-line-number" data-line-number="1351"></td>
        <td id="LC1351" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L1352" class="blob-num js-line-number" data-line-number="1352"></td>
        <td id="LC1352" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1353" class="blob-num js-line-number" data-line-number="1353"></td>
        <td id="LC1353" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1354" class="blob-num js-line-number" data-line-number="1354"></td>
        <td id="LC1354" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \brief  Clear Pending Interrupt</span></td>
      </tr>
      <tr>
        <td id="L1355" class="blob-num js-line-number" data-line-number="1355"></td>
        <td id="LC1355" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L1356" class="blob-num js-line-number" data-line-number="1356"></td>
        <td id="LC1356" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    The function clears the pending bit of an external interrupt.</span></td>
      </tr>
      <tr>
        <td id="L1357" class="blob-num js-line-number" data-line-number="1357"></td>
        <td id="LC1357" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L1358" class="blob-num js-line-number" data-line-number="1358"></td>
        <td id="LC1358" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \param [in]      IRQn  External interrupt number. Value cannot be negative.</span></td>
      </tr>
      <tr>
        <td id="L1359" class="blob-num js-line-number" data-line-number="1359"></td>
        <td id="LC1359" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1360" class="blob-num js-line-number" data-line-number="1360"></td>
        <td id="LC1360" class="blob-code blob-code-inner js-file-line">__STATIC_INLINE <span class="pl-k">void</span> <span class="pl-smi">NVIC_ClearPendingIRQ</span>(IRQn_Type IRQn)</td>
      </tr>
      <tr>
        <td id="L1361" class="blob-num js-line-number" data-line-number="1361"></td>
        <td id="LC1361" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L1362" class="blob-num js-line-number" data-line-number="1362"></td>
        <td id="LC1362" class="blob-code blob-code-inner js-file-line">  NVIC-&gt;ICPR[((<span class="pl-c1">uint32_t</span>)(IRQn) &gt;&gt; <span class="pl-c1">5</span>)] = (<span class="pl-c1">1</span> &lt;&lt; ((<span class="pl-c1">uint32_t</span>)(IRQn) &amp; 0x1F)); <span class="pl-c"><span class="pl-c">/*</span> Clear pending interrupt <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1363" class="blob-num js-line-number" data-line-number="1363"></td>
        <td id="LC1363" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L1364" class="blob-num js-line-number" data-line-number="1364"></td>
        <td id="LC1364" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1365" class="blob-num js-line-number" data-line-number="1365"></td>
        <td id="LC1365" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1366" class="blob-num js-line-number" data-line-number="1366"></td>
        <td id="LC1366" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \brief  Get Active Interrupt</span></td>
      </tr>
      <tr>
        <td id="L1367" class="blob-num js-line-number" data-line-number="1367"></td>
        <td id="LC1367" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L1368" class="blob-num js-line-number" data-line-number="1368"></td>
        <td id="LC1368" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    The function reads the active register in NVIC and returns the active bit.</span></td>
      </tr>
      <tr>
        <td id="L1369" class="blob-num js-line-number" data-line-number="1369"></td>
        <td id="LC1369" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L1370" class="blob-num js-line-number" data-line-number="1370"></td>
        <td id="LC1370" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \param [in]      IRQn  Interrupt number.</span></td>
      </tr>
      <tr>
        <td id="L1371" class="blob-num js-line-number" data-line-number="1371"></td>
        <td id="LC1371" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L1372" class="blob-num js-line-number" data-line-number="1372"></td>
        <td id="LC1372" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \return             0  Interrupt status is not active.</span></td>
      </tr>
      <tr>
        <td id="L1373" class="blob-num js-line-number" data-line-number="1373"></td>
        <td id="LC1373" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \return             1  Interrupt status is active.</span></td>
      </tr>
      <tr>
        <td id="L1374" class="blob-num js-line-number" data-line-number="1374"></td>
        <td id="LC1374" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1375" class="blob-num js-line-number" data-line-number="1375"></td>
        <td id="LC1375" class="blob-code blob-code-inner js-file-line">__STATIC_INLINE <span class="pl-c1">uint32_t</span> <span class="pl-smi">NVIC_GetActive</span>(IRQn_Type IRQn)</td>
      </tr>
      <tr>
        <td id="L1376" class="blob-num js-line-number" data-line-number="1376"></td>
        <td id="LC1376" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L1377" class="blob-num js-line-number" data-line-number="1377"></td>
        <td id="LC1377" class="blob-code blob-code-inner js-file-line">  <span class="pl-k">return</span>((<span class="pl-c1">uint32_t</span>)((NVIC-&gt;IABR[(<span class="pl-c1">uint32_t</span>)(IRQn) &gt;&gt; <span class="pl-c1">5</span>] &amp; (<span class="pl-c1">1</span> &lt;&lt; ((<span class="pl-c1">uint32_t</span>)(IRQn) &amp; 0x1F)))?<span class="pl-c1">1</span>:<span class="pl-c1">0</span>)); <span class="pl-c"><span class="pl-c">/*</span> Return 1 if active else 0 <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1378" class="blob-num js-line-number" data-line-number="1378"></td>
        <td id="LC1378" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L1379" class="blob-num js-line-number" data-line-number="1379"></td>
        <td id="LC1379" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1380" class="blob-num js-line-number" data-line-number="1380"></td>
        <td id="LC1380" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1381" class="blob-num js-line-number" data-line-number="1381"></td>
        <td id="LC1381" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \brief  Set Interrupt Priority</span></td>
      </tr>
      <tr>
        <td id="L1382" class="blob-num js-line-number" data-line-number="1382"></td>
        <td id="LC1382" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L1383" class="blob-num js-line-number" data-line-number="1383"></td>
        <td id="LC1383" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    The function sets the priority of an interrupt.</span></td>
      </tr>
      <tr>
        <td id="L1384" class="blob-num js-line-number" data-line-number="1384"></td>
        <td id="LC1384" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L1385" class="blob-num js-line-number" data-line-number="1385"></td>
        <td id="LC1385" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \note The priority cannot be set for every core interrupt.</span></td>
      </tr>
      <tr>
        <td id="L1386" class="blob-num js-line-number" data-line-number="1386"></td>
        <td id="LC1386" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L1387" class="blob-num js-line-number" data-line-number="1387"></td>
        <td id="LC1387" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \param [in]      IRQn  Interrupt number.</span></td>
      </tr>
      <tr>
        <td id="L1388" class="blob-num js-line-number" data-line-number="1388"></td>
        <td id="LC1388" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \param [in]  priority  Priority to set.</span></td>
      </tr>
      <tr>
        <td id="L1389" class="blob-num js-line-number" data-line-number="1389"></td>
        <td id="LC1389" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1390" class="blob-num js-line-number" data-line-number="1390"></td>
        <td id="LC1390" class="blob-code blob-code-inner js-file-line">__STATIC_INLINE <span class="pl-k">void</span> <span class="pl-smi">NVIC_SetPriority</span>(IRQn_Type IRQn, <span class="pl-c1">uint32_t</span> priority)</td>
      </tr>
      <tr>
        <td id="L1391" class="blob-num js-line-number" data-line-number="1391"></td>
        <td id="LC1391" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L1392" class="blob-num js-line-number" data-line-number="1392"></td>
        <td id="LC1392" class="blob-code blob-code-inner js-file-line">  <span class="pl-k">if</span>(IRQn &lt; <span class="pl-c1">0</span>) {</td>
      </tr>
      <tr>
        <td id="L1393" class="blob-num js-line-number" data-line-number="1393"></td>
        <td id="LC1393" class="blob-code blob-code-inner js-file-line">    SCB-&gt;SHP[((<span class="pl-c1">uint32_t</span>)(IRQn) &amp; <span class="pl-c1">0xF</span>)-<span class="pl-c1">4</span>] = ((priority &lt;&lt; (<span class="pl-c1">8</span> - __NVIC_PRIO_BITS)) &amp; 0xff); } <span class="pl-c"><span class="pl-c">/*</span> set Priority for Cortex-M  System Interrupts <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1394" class="blob-num js-line-number" data-line-number="1394"></td>
        <td id="LC1394" class="blob-code blob-code-inner js-file-line">  <span class="pl-k">else</span> {</td>
      </tr>
      <tr>
        <td id="L1395" class="blob-num js-line-number" data-line-number="1395"></td>
        <td id="LC1395" class="blob-code blob-code-inner js-file-line">    NVIC-&gt;IP[(<span class="pl-c1">uint32_t</span>)(IRQn)] = ((priority &lt;&lt; (<span class="pl-c1">8</span> - __NVIC_PRIO_BITS)) &amp; 0xff);    }        <span class="pl-c"><span class="pl-c">/*</span> set Priority for device specific Interrupts  <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1396" class="blob-num js-line-number" data-line-number="1396"></td>
        <td id="LC1396" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L1397" class="blob-num js-line-number" data-line-number="1397"></td>
        <td id="LC1397" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1398" class="blob-num js-line-number" data-line-number="1398"></td>
        <td id="LC1398" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1399" class="blob-num js-line-number" data-line-number="1399"></td>
        <td id="LC1399" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \brief  Get Interrupt Priority</span></td>
      </tr>
      <tr>
        <td id="L1400" class="blob-num js-line-number" data-line-number="1400"></td>
        <td id="LC1400" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L1401" class="blob-num js-line-number" data-line-number="1401"></td>
        <td id="LC1401" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    The function reads the priority of an interrupt. The interrupt</span></td>
      </tr>
      <tr>
        <td id="L1402" class="blob-num js-line-number" data-line-number="1402"></td>
        <td id="LC1402" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    number can be positive to specify an external (device specific)</span></td>
      </tr>
      <tr>
        <td id="L1403" class="blob-num js-line-number" data-line-number="1403"></td>
        <td id="LC1403" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    interrupt, or negative to specify an internal (core) interrupt.</span></td>
      </tr>
      <tr>
        <td id="L1404" class="blob-num js-line-number" data-line-number="1404"></td>
        <td id="LC1404" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L1405" class="blob-num js-line-number" data-line-number="1405"></td>
        <td id="LC1405" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L1406" class="blob-num js-line-number" data-line-number="1406"></td>
        <td id="LC1406" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \param [in]   IRQn  Interrupt number.</span></td>
      </tr>
      <tr>
        <td id="L1407" class="blob-num js-line-number" data-line-number="1407"></td>
        <td id="LC1407" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \return             Interrupt Priority. Value is aligned automatically to the implemented</span></td>
      </tr>
      <tr>
        <td id="L1408" class="blob-num js-line-number" data-line-number="1408"></td>
        <td id="LC1408" class="blob-code blob-code-inner js-file-line"><span class="pl-c">                        priority bits of the microcontroller.</span></td>
      </tr>
      <tr>
        <td id="L1409" class="blob-num js-line-number" data-line-number="1409"></td>
        <td id="LC1409" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1410" class="blob-num js-line-number" data-line-number="1410"></td>
        <td id="LC1410" class="blob-code blob-code-inner js-file-line">__STATIC_INLINE <span class="pl-c1">uint32_t</span> <span class="pl-smi">NVIC_GetPriority</span>(IRQn_Type IRQn)</td>
      </tr>
      <tr>
        <td id="L1411" class="blob-num js-line-number" data-line-number="1411"></td>
        <td id="LC1411" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L1412" class="blob-num js-line-number" data-line-number="1412"></td>
        <td id="LC1412" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1413" class="blob-num js-line-number" data-line-number="1413"></td>
        <td id="LC1413" class="blob-code blob-code-inner js-file-line">  <span class="pl-k">if</span>(IRQn &lt; <span class="pl-c1">0</span>) {</td>
      </tr>
      <tr>
        <td id="L1414" class="blob-num js-line-number" data-line-number="1414"></td>
        <td id="LC1414" class="blob-code blob-code-inner js-file-line">    <span class="pl-k">return</span>((<span class="pl-c1">uint32_t</span>)(SCB-&gt;SHP[((<span class="pl-c1">uint32_t</span>)(IRQn) &amp; <span class="pl-c1">0xF</span>)-<span class="pl-c1">4</span>] &gt;&gt; (<span class="pl-c1">8</span> - __NVIC_PRIO_BITS)));  } <span class="pl-c"><span class="pl-c">/*</span> get priority for Cortex-M  system interrupts <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1415" class="blob-num js-line-number" data-line-number="1415"></td>
        <td id="LC1415" class="blob-code blob-code-inner js-file-line">  <span class="pl-k">else</span> {</td>
      </tr>
      <tr>
        <td id="L1416" class="blob-num js-line-number" data-line-number="1416"></td>
        <td id="LC1416" class="blob-code blob-code-inner js-file-line">    <span class="pl-k">return</span>((<span class="pl-c1">uint32_t</span>)(NVIC-&gt;IP[(<span class="pl-c1">uint32_t</span>)(IRQn)]           &gt;&gt; (<span class="pl-c1">8</span> - __NVIC_PRIO_BITS)));  } <span class="pl-c"><span class="pl-c">/*</span> get priority for device specific interrupts  <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1417" class="blob-num js-line-number" data-line-number="1417"></td>
        <td id="LC1417" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L1418" class="blob-num js-line-number" data-line-number="1418"></td>
        <td id="LC1418" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1419" class="blob-num js-line-number" data-line-number="1419"></td>
        <td id="LC1419" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1420" class="blob-num js-line-number" data-line-number="1420"></td>
        <td id="LC1420" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \brief  Encode Priority</span></td>
      </tr>
      <tr>
        <td id="L1421" class="blob-num js-line-number" data-line-number="1421"></td>
        <td id="LC1421" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L1422" class="blob-num js-line-number" data-line-number="1422"></td>
        <td id="LC1422" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    The function encodes the priority for an interrupt with the given priority group,</span></td>
      </tr>
      <tr>
        <td id="L1423" class="blob-num js-line-number" data-line-number="1423"></td>
        <td id="LC1423" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    preemptive priority value, and subpriority value.</span></td>
      </tr>
      <tr>
        <td id="L1424" class="blob-num js-line-number" data-line-number="1424"></td>
        <td id="LC1424" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    In case of a conflict between priority grouping and available</span></td>
      </tr>
      <tr>
        <td id="L1425" class="blob-num js-line-number" data-line-number="1425"></td>
        <td id="LC1425" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    priority bits (__NVIC_PRIO_BITS), the samllest possible priority group is set.</span></td>
      </tr>
      <tr>
        <td id="L1426" class="blob-num js-line-number" data-line-number="1426"></td>
        <td id="LC1426" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L1427" class="blob-num js-line-number" data-line-number="1427"></td>
        <td id="LC1427" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \param [in]     PriorityGroup  Used priority group.</span></td>
      </tr>
      <tr>
        <td id="L1428" class="blob-num js-line-number" data-line-number="1428"></td>
        <td id="LC1428" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \param [in]   PreemptPriority  Preemptive priority value (starting from 0).</span></td>
      </tr>
      <tr>
        <td id="L1429" class="blob-num js-line-number" data-line-number="1429"></td>
        <td id="LC1429" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \param [in]       SubPriority  Subpriority value (starting from 0).</span></td>
      </tr>
      <tr>
        <td id="L1430" class="blob-num js-line-number" data-line-number="1430"></td>
        <td id="LC1430" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \return                        Encoded priority. Value can be used in the function \ref NVIC_SetPriority().</span></td>
      </tr>
      <tr>
        <td id="L1431" class="blob-num js-line-number" data-line-number="1431"></td>
        <td id="LC1431" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1432" class="blob-num js-line-number" data-line-number="1432"></td>
        <td id="LC1432" class="blob-code blob-code-inner js-file-line">__STATIC_INLINE <span class="pl-c1">uint32_t</span> <span class="pl-smi">NVIC_EncodePriority</span> (<span class="pl-c1">uint32_t</span> PriorityGroup, <span class="pl-c1">uint32_t</span> PreemptPriority, <span class="pl-c1">uint32_t</span> SubPriority)</td>
      </tr>
      <tr>
        <td id="L1433" class="blob-num js-line-number" data-line-number="1433"></td>
        <td id="LC1433" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L1434" class="blob-num js-line-number" data-line-number="1434"></td>
        <td id="LC1434" class="blob-code blob-code-inner js-file-line">  <span class="pl-c1">uint32_t</span> PriorityGroupTmp = (PriorityGroup &amp; 0x07);          <span class="pl-c"><span class="pl-c">/*</span> only values 0..7 are used          <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1435" class="blob-num js-line-number" data-line-number="1435"></td>
        <td id="LC1435" class="blob-code blob-code-inner js-file-line">  <span class="pl-c1">uint32_t</span> PreemptPriorityBits;</td>
      </tr>
      <tr>
        <td id="L1436" class="blob-num js-line-number" data-line-number="1436"></td>
        <td id="LC1436" class="blob-code blob-code-inner js-file-line">  <span class="pl-c1">uint32_t</span> SubPriorityBits;</td>
      </tr>
      <tr>
        <td id="L1437" class="blob-num js-line-number" data-line-number="1437"></td>
        <td id="LC1437" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1438" class="blob-num js-line-number" data-line-number="1438"></td>
        <td id="LC1438" class="blob-code blob-code-inner js-file-line">  PreemptPriorityBits = ((<span class="pl-c1">7</span> - PriorityGroupTmp) &gt; __NVIC_PRIO_BITS) ? __NVIC_PRIO_BITS : <span class="pl-c1">7</span> - PriorityGroupTmp;</td>
      </tr>
      <tr>
        <td id="L1439" class="blob-num js-line-number" data-line-number="1439"></td>
        <td id="LC1439" class="blob-code blob-code-inner js-file-line">  SubPriorityBits     = ((PriorityGroupTmp + __NVIC_PRIO_BITS) &lt; <span class="pl-c1">7</span>) ? <span class="pl-c1">0</span> : PriorityGroupTmp - <span class="pl-c1">7</span> + __NVIC_PRIO_BITS;</td>
      </tr>
      <tr>
        <td id="L1440" class="blob-num js-line-number" data-line-number="1440"></td>
        <td id="LC1440" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1441" class="blob-num js-line-number" data-line-number="1441"></td>
        <td id="LC1441" class="blob-code blob-code-inner js-file-line">  <span class="pl-k">return</span> (</td>
      </tr>
      <tr>
        <td id="L1442" class="blob-num js-line-number" data-line-number="1442"></td>
        <td id="LC1442" class="blob-code blob-code-inner js-file-line">           ((PreemptPriority &amp; ((<span class="pl-c1">1</span> &lt;&lt; (PreemptPriorityBits)) - <span class="pl-c1">1</span>)) &lt;&lt; SubPriorityBits) |</td>
      </tr>
      <tr>
        <td id="L1443" class="blob-num js-line-number" data-line-number="1443"></td>
        <td id="LC1443" class="blob-code blob-code-inner js-file-line">           ((SubPriority     &amp; ((<span class="pl-c1">1</span> &lt;&lt; (SubPriorityBits    )) - <span class="pl-c1">1</span>)))</td>
      </tr>
      <tr>
        <td id="L1444" class="blob-num js-line-number" data-line-number="1444"></td>
        <td id="LC1444" class="blob-code blob-code-inner js-file-line">         );</td>
      </tr>
      <tr>
        <td id="L1445" class="blob-num js-line-number" data-line-number="1445"></td>
        <td id="LC1445" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L1446" class="blob-num js-line-number" data-line-number="1446"></td>
        <td id="LC1446" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1447" class="blob-num js-line-number" data-line-number="1447"></td>
        <td id="LC1447" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1448" class="blob-num js-line-number" data-line-number="1448"></td>
        <td id="LC1448" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \brief  Decode Priority</span></td>
      </tr>
      <tr>
        <td id="L1449" class="blob-num js-line-number" data-line-number="1449"></td>
        <td id="LC1449" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L1450" class="blob-num js-line-number" data-line-number="1450"></td>
        <td id="LC1450" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    The function decodes an interrupt priority value with a given priority group to</span></td>
      </tr>
      <tr>
        <td id="L1451" class="blob-num js-line-number" data-line-number="1451"></td>
        <td id="LC1451" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    preemptive priority value and subpriority value.</span></td>
      </tr>
      <tr>
        <td id="L1452" class="blob-num js-line-number" data-line-number="1452"></td>
        <td id="LC1452" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    In case of a conflict between priority grouping and available</span></td>
      </tr>
      <tr>
        <td id="L1453" class="blob-num js-line-number" data-line-number="1453"></td>
        <td id="LC1453" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    priority bits (__NVIC_PRIO_BITS) the samllest possible priority group is set.</span></td>
      </tr>
      <tr>
        <td id="L1454" class="blob-num js-line-number" data-line-number="1454"></td>
        <td id="LC1454" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L1455" class="blob-num js-line-number" data-line-number="1455"></td>
        <td id="LC1455" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \param [in]         Priority   Priority value, which can be retrieved with the function \ref NVIC_GetPriority().</span></td>
      </tr>
      <tr>
        <td id="L1456" class="blob-num js-line-number" data-line-number="1456"></td>
        <td id="LC1456" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \param [in]     PriorityGroup  Used priority group.</span></td>
      </tr>
      <tr>
        <td id="L1457" class="blob-num js-line-number" data-line-number="1457"></td>
        <td id="LC1457" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \param [out] pPreemptPriority  Preemptive priority value (starting from 0).</span></td>
      </tr>
      <tr>
        <td id="L1458" class="blob-num js-line-number" data-line-number="1458"></td>
        <td id="LC1458" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \param [out]     pSubPriority  Subpriority value (starting from 0).</span></td>
      </tr>
      <tr>
        <td id="L1459" class="blob-num js-line-number" data-line-number="1459"></td>
        <td id="LC1459" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1460" class="blob-num js-line-number" data-line-number="1460"></td>
        <td id="LC1460" class="blob-code blob-code-inner js-file-line">__STATIC_INLINE <span class="pl-k">void</span> <span class="pl-smi">NVIC_DecodePriority</span> (<span class="pl-c1">uint32_t</span> Priority, <span class="pl-c1">uint32_t</span> PriorityGroup, <span class="pl-c1">uint32_t</span>* pPreemptPriority, <span class="pl-c1">uint32_t</span>* pSubPriority)</td>
      </tr>
      <tr>
        <td id="L1461" class="blob-num js-line-number" data-line-number="1461"></td>
        <td id="LC1461" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L1462" class="blob-num js-line-number" data-line-number="1462"></td>
        <td id="LC1462" class="blob-code blob-code-inner js-file-line">  <span class="pl-c1">uint32_t</span> PriorityGroupTmp = (PriorityGroup &amp; 0x07);          <span class="pl-c"><span class="pl-c">/*</span> only values 0..7 are used          <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1463" class="blob-num js-line-number" data-line-number="1463"></td>
        <td id="LC1463" class="blob-code blob-code-inner js-file-line">  <span class="pl-c1">uint32_t</span> PreemptPriorityBits;</td>
      </tr>
      <tr>
        <td id="L1464" class="blob-num js-line-number" data-line-number="1464"></td>
        <td id="LC1464" class="blob-code blob-code-inner js-file-line">  <span class="pl-c1">uint32_t</span> SubPriorityBits;</td>
      </tr>
      <tr>
        <td id="L1465" class="blob-num js-line-number" data-line-number="1465"></td>
        <td id="LC1465" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1466" class="blob-num js-line-number" data-line-number="1466"></td>
        <td id="LC1466" class="blob-code blob-code-inner js-file-line">  PreemptPriorityBits = ((<span class="pl-c1">7</span> - PriorityGroupTmp) &gt; __NVIC_PRIO_BITS) ? __NVIC_PRIO_BITS : <span class="pl-c1">7</span> - PriorityGroupTmp;</td>
      </tr>
      <tr>
        <td id="L1467" class="blob-num js-line-number" data-line-number="1467"></td>
        <td id="LC1467" class="blob-code blob-code-inner js-file-line">  SubPriorityBits     = ((PriorityGroupTmp + __NVIC_PRIO_BITS) &lt; <span class="pl-c1">7</span>) ? <span class="pl-c1">0</span> : PriorityGroupTmp - <span class="pl-c1">7</span> + __NVIC_PRIO_BITS;</td>
      </tr>
      <tr>
        <td id="L1468" class="blob-num js-line-number" data-line-number="1468"></td>
        <td id="LC1468" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1469" class="blob-num js-line-number" data-line-number="1469"></td>
        <td id="LC1469" class="blob-code blob-code-inner js-file-line">  *pPreemptPriority = (Priority &gt;&gt; SubPriorityBits) &amp; ((<span class="pl-c1">1</span> &lt;&lt; (PreemptPriorityBits)) - <span class="pl-c1">1</span>);</td>
      </tr>
      <tr>
        <td id="L1470" class="blob-num js-line-number" data-line-number="1470"></td>
        <td id="LC1470" class="blob-code blob-code-inner js-file-line">  *pSubPriority     = (Priority                   ) &amp; ((<span class="pl-c1">1</span> &lt;&lt; (SubPriorityBits    )) - <span class="pl-c1">1</span>);</td>
      </tr>
      <tr>
        <td id="L1471" class="blob-num js-line-number" data-line-number="1471"></td>
        <td id="LC1471" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L1472" class="blob-num js-line-number" data-line-number="1472"></td>
        <td id="LC1472" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1473" class="blob-num js-line-number" data-line-number="1473"></td>
        <td id="LC1473" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1474" class="blob-num js-line-number" data-line-number="1474"></td>
        <td id="LC1474" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \brief  System Reset</span></td>
      </tr>
      <tr>
        <td id="L1475" class="blob-num js-line-number" data-line-number="1475"></td>
        <td id="LC1475" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L1476" class="blob-num js-line-number" data-line-number="1476"></td>
        <td id="LC1476" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    The function initiates a system reset request to reset the MCU.</span></td>
      </tr>
      <tr>
        <td id="L1477" class="blob-num js-line-number" data-line-number="1477"></td>
        <td id="LC1477" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1478" class="blob-num js-line-number" data-line-number="1478"></td>
        <td id="LC1478" class="blob-code blob-code-inner js-file-line">__STATIC_INLINE <span class="pl-k">void</span> <span class="pl-smi">NVIC_SystemReset</span>(<span class="pl-k">void</span>)</td>
      </tr>
      <tr>
        <td id="L1479" class="blob-num js-line-number" data-line-number="1479"></td>
        <td id="LC1479" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L1480" class="blob-num js-line-number" data-line-number="1480"></td>
        <td id="LC1480" class="blob-code blob-code-inner js-file-line">  <span class="pl-c1">__DSB</span>();                                                     <span class="pl-c"><span class="pl-c">/*</span> Ensure all outstanding memory accesses included</span></td>
      </tr>
      <tr>
        <td id="L1481" class="blob-num js-line-number" data-line-number="1481"></td>
        <td id="LC1481" class="blob-code blob-code-inner js-file-line"><span class="pl-c">                                                                  buffered write are completed before reset <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1482" class="blob-num js-line-number" data-line-number="1482"></td>
        <td id="LC1482" class="blob-code blob-code-inner js-file-line">  SCB-&gt;AIRCR  = ((0x5FA &lt;&lt; SCB_AIRCR_VECTKEY_Pos)      |</td>
      </tr>
      <tr>
        <td id="L1483" class="blob-num js-line-number" data-line-number="1483"></td>
        <td id="LC1483" class="blob-code blob-code-inner js-file-line">                 (SCB-&gt;AIRCR &amp; SCB_AIRCR_PRIGROUP_Msk) |</td>
      </tr>
      <tr>
        <td id="L1484" class="blob-num js-line-number" data-line-number="1484"></td>
        <td id="LC1484" class="blob-code blob-code-inner js-file-line">                 SCB_AIRCR_SYSRESETREQ_Msk);                   <span class="pl-c"><span class="pl-c">/*</span> Keep priority group unchanged <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1485" class="blob-num js-line-number" data-line-number="1485"></td>
        <td id="LC1485" class="blob-code blob-code-inner js-file-line">  <span class="pl-c1">__DSB</span>();                                                     <span class="pl-c"><span class="pl-c">/*</span> Ensure completion of memory access <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1486" class="blob-num js-line-number" data-line-number="1486"></td>
        <td id="LC1486" class="blob-code blob-code-inner js-file-line">  <span class="pl-k">while</span>(<span class="pl-c1">1</span>);                                                    <span class="pl-c"><span class="pl-c">/*</span> wait until reset <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1487" class="blob-num js-line-number" data-line-number="1487"></td>
        <td id="LC1487" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L1488" class="blob-num js-line-number" data-line-number="1488"></td>
        <td id="LC1488" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1489" class="blob-num js-line-number" data-line-number="1489"></td>
        <td id="LC1489" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>@} end of CMSIS_Core_NVICFunctions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1490" class="blob-num js-line-number" data-line-number="1490"></td>
        <td id="LC1490" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1491" class="blob-num js-line-number" data-line-number="1491"></td>
        <td id="LC1491" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1492" class="blob-num js-line-number" data-line-number="1492"></td>
        <td id="LC1492" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1493" class="blob-num js-line-number" data-line-number="1493"></td>
        <td id="LC1493" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> ##################################    SysTick function  ############################################ <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1494" class="blob-num js-line-number" data-line-number="1494"></td>
        <td id="LC1494" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \ingroup  CMSIS_Core_FunctionInterface</span></td>
      </tr>
      <tr>
        <td id="L1495" class="blob-num js-line-number" data-line-number="1495"></td>
        <td id="LC1495" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \defgroup CMSIS_Core_SysTickFunctions SysTick Functions</span></td>
      </tr>
      <tr>
        <td id="L1496" class="blob-num js-line-number" data-line-number="1496"></td>
        <td id="LC1496" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \brief      Functions that configure the System.</span></td>
      </tr>
      <tr>
        <td id="L1497" class="blob-num js-line-number" data-line-number="1497"></td>
        <td id="LC1497" class="blob-code blob-code-inner js-file-line"><span class="pl-c">  @{</span></td>
      </tr>
      <tr>
        <td id="L1498" class="blob-num js-line-number" data-line-number="1498"></td>
        <td id="LC1498" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1499" class="blob-num js-line-number" data-line-number="1499"></td>
        <td id="LC1499" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1500" class="blob-num js-line-number" data-line-number="1500"></td>
        <td id="LC1500" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">if</span> (__Vendor_SysTickConfig == 0)</td>
      </tr>
      <tr>
        <td id="L1501" class="blob-num js-line-number" data-line-number="1501"></td>
        <td id="LC1501" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1502" class="blob-num js-line-number" data-line-number="1502"></td>
        <td id="LC1502" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \brief  System Tick Configuration</span></td>
      </tr>
      <tr>
        <td id="L1503" class="blob-num js-line-number" data-line-number="1503"></td>
        <td id="LC1503" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L1504" class="blob-num js-line-number" data-line-number="1504"></td>
        <td id="LC1504" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    The function initializes the System Timer and its interrupt, and starts the System Tick Timer.</span></td>
      </tr>
      <tr>
        <td id="L1505" class="blob-num js-line-number" data-line-number="1505"></td>
        <td id="LC1505" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    Counter is in free running mode to generate periodic interrupts.</span></td>
      </tr>
      <tr>
        <td id="L1506" class="blob-num js-line-number" data-line-number="1506"></td>
        <td id="LC1506" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L1507" class="blob-num js-line-number" data-line-number="1507"></td>
        <td id="LC1507" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \param [in]  ticks  Number of ticks between two interrupts.</span></td>
      </tr>
      <tr>
        <td id="L1508" class="blob-num js-line-number" data-line-number="1508"></td>
        <td id="LC1508" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L1509" class="blob-num js-line-number" data-line-number="1509"></td>
        <td id="LC1509" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \return          0  Function succeeded.</span></td>
      </tr>
      <tr>
        <td id="L1510" class="blob-num js-line-number" data-line-number="1510"></td>
        <td id="LC1510" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \return          1  Function failed.</span></td>
      </tr>
      <tr>
        <td id="L1511" class="blob-num js-line-number" data-line-number="1511"></td>
        <td id="LC1511" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L1512" class="blob-num js-line-number" data-line-number="1512"></td>
        <td id="LC1512" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \note     When the variable &lt;b&gt;__Vendor_SysTickConfig&lt;/b&gt; is set to 1, then the</span></td>
      </tr>
      <tr>
        <td id="L1513" class="blob-num js-line-number" data-line-number="1513"></td>
        <td id="LC1513" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    function &lt;b&gt;SysTick_Config&lt;/b&gt; is not included. In this case, the file &lt;b&gt;&lt;i&gt;device&lt;/i&gt;.h&lt;/b&gt;</span></td>
      </tr>
      <tr>
        <td id="L1514" class="blob-num js-line-number" data-line-number="1514"></td>
        <td id="LC1514" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    must contain a vendor-specific implementation of this function.</span></td>
      </tr>
      <tr>
        <td id="L1515" class="blob-num js-line-number" data-line-number="1515"></td>
        <td id="LC1515" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L1516" class="blob-num js-line-number" data-line-number="1516"></td>
        <td id="LC1516" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1517" class="blob-num js-line-number" data-line-number="1517"></td>
        <td id="LC1517" class="blob-code blob-code-inner js-file-line">__STATIC_INLINE <span class="pl-c1">uint32_t</span> <span class="pl-smi">SysTick_Config</span>(<span class="pl-c1">uint32_t</span> ticks)</td>
      </tr>
      <tr>
        <td id="L1518" class="blob-num js-line-number" data-line-number="1518"></td>
        <td id="LC1518" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L1519" class="blob-num js-line-number" data-line-number="1519"></td>
        <td id="LC1519" class="blob-code blob-code-inner js-file-line">  <span class="pl-k">if</span> (ticks &gt; SysTick_LOAD_RELOAD_Msk)  <span class="pl-k">return</span> (<span class="pl-c1">1</span>);            <span class="pl-c"><span class="pl-c">/*</span> Reload value impossible <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1520" class="blob-num js-line-number" data-line-number="1520"></td>
        <td id="LC1520" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1521" class="blob-num js-line-number" data-line-number="1521"></td>
        <td id="LC1521" class="blob-code blob-code-inner js-file-line">  SysTick-&gt;LOAD  = (ticks &amp; SysTick_LOAD_RELOAD_Msk) - <span class="pl-c1">1</span>;      <span class="pl-c"><span class="pl-c">/*</span> set reload register <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1522" class="blob-num js-line-number" data-line-number="1522"></td>
        <td id="LC1522" class="blob-code blob-code-inner js-file-line">  <span class="pl-c1">NVIC_SetPriority</span> (SysTick_IRQn, (<span class="pl-c1">1</span>&lt;&lt;__NVIC_PRIO_BITS) - <span class="pl-c1">1</span>);  <span class="pl-c"><span class="pl-c">/*</span> set Priority for Systick Interrupt <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1523" class="blob-num js-line-number" data-line-number="1523"></td>
        <td id="LC1523" class="blob-code blob-code-inner js-file-line">  SysTick-&gt;VAL   = <span class="pl-c1">0</span>;                                          <span class="pl-c"><span class="pl-c">/*</span> Load the SysTick Counter Value <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1524" class="blob-num js-line-number" data-line-number="1524"></td>
        <td id="LC1524" class="blob-code blob-code-inner js-file-line">  SysTick-&gt;CTRL  = SysTick_CTRL_CLKSOURCE_Msk |</td>
      </tr>
      <tr>
        <td id="L1525" class="blob-num js-line-number" data-line-number="1525"></td>
        <td id="LC1525" class="blob-code blob-code-inner js-file-line">                   SysTick_CTRL_TICKINT_Msk   |</td>
      </tr>
      <tr>
        <td id="L1526" class="blob-num js-line-number" data-line-number="1526"></td>
        <td id="LC1526" class="blob-code blob-code-inner js-file-line">                   SysTick_CTRL_ENABLE_Msk;                    <span class="pl-c"><span class="pl-c">/*</span> Enable SysTick IRQ and SysTick Timer <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1527" class="blob-num js-line-number" data-line-number="1527"></td>
        <td id="LC1527" class="blob-code blob-code-inner js-file-line">  <span class="pl-k">return</span> (<span class="pl-c1">0</span>);                                                  <span class="pl-c"><span class="pl-c">/*</span> Function successful <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1528" class="blob-num js-line-number" data-line-number="1528"></td>
        <td id="LC1528" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L1529" class="blob-num js-line-number" data-line-number="1529"></td>
        <td id="LC1529" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1530" class="blob-num js-line-number" data-line-number="1530"></td>
        <td id="LC1530" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">endif</span></td>
      </tr>
      <tr>
        <td id="L1531" class="blob-num js-line-number" data-line-number="1531"></td>
        <td id="LC1531" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1532" class="blob-num js-line-number" data-line-number="1532"></td>
        <td id="LC1532" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>@} end of CMSIS_Core_SysTickFunctions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1533" class="blob-num js-line-number" data-line-number="1533"></td>
        <td id="LC1533" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1534" class="blob-num js-line-number" data-line-number="1534"></td>
        <td id="LC1534" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1535" class="blob-num js-line-number" data-line-number="1535"></td>
        <td id="LC1535" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1536" class="blob-num js-line-number" data-line-number="1536"></td>
        <td id="LC1536" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> ##################################### Debug In/Output function ########################################### <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1537" class="blob-num js-line-number" data-line-number="1537"></td>
        <td id="LC1537" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \ingroup  CMSIS_Core_FunctionInterface</span></td>
      </tr>
      <tr>
        <td id="L1538" class="blob-num js-line-number" data-line-number="1538"></td>
        <td id="LC1538" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \defgroup CMSIS_core_DebugFunctions ITM Functions</span></td>
      </tr>
      <tr>
        <td id="L1539" class="blob-num js-line-number" data-line-number="1539"></td>
        <td id="LC1539" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \brief   Functions that access the ITM debug interface.</span></td>
      </tr>
      <tr>
        <td id="L1540" class="blob-num js-line-number" data-line-number="1540"></td>
        <td id="LC1540" class="blob-code blob-code-inner js-file-line"><span class="pl-c">  @{</span></td>
      </tr>
      <tr>
        <td id="L1541" class="blob-num js-line-number" data-line-number="1541"></td>
        <td id="LC1541" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1542" class="blob-num js-line-number" data-line-number="1542"></td>
        <td id="LC1542" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1543" class="blob-num js-line-number" data-line-number="1543"></td>
        <td id="LC1543" class="blob-code blob-code-inner js-file-line"><span class="pl-k">extern</span> <span class="pl-k">volatile</span> <span class="pl-c1">int32_t</span> ITM_RxBuffer;                    <span class="pl-c"><span class="pl-c">/*</span>!&lt; External variable to receive characters.                         <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1544" class="blob-num js-line-number" data-line-number="1544"></td>
        <td id="LC1544" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span>                 <span class="pl-en">ITM_RXBUFFER_EMPTY</span>    0x5AA55AA5 <span class="pl-c"><span class="pl-c">/*</span>!&lt; Value identifying \ref ITM_RxBuffer is ready for next character. <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1545" class="blob-num js-line-number" data-line-number="1545"></td>
        <td id="LC1545" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1546" class="blob-num js-line-number" data-line-number="1546"></td>
        <td id="LC1546" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1547" class="blob-num js-line-number" data-line-number="1547"></td>
        <td id="LC1547" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \brief  ITM Send Character</span></td>
      </tr>
      <tr>
        <td id="L1548" class="blob-num js-line-number" data-line-number="1548"></td>
        <td id="LC1548" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L1549" class="blob-num js-line-number" data-line-number="1549"></td>
        <td id="LC1549" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    The function transmits a character via the ITM channel 0, and</span></td>
      </tr>
      <tr>
        <td id="L1550" class="blob-num js-line-number" data-line-number="1550"></td>
        <td id="LC1550" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \li Just returns when no debugger is connected that has booked the output.</span></td>
      </tr>
      <tr>
        <td id="L1551" class="blob-num js-line-number" data-line-number="1551"></td>
        <td id="LC1551" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \li Is blocking when a debugger is connected, but the previous character sent has not been transmitted.</span></td>
      </tr>
      <tr>
        <td id="L1552" class="blob-num js-line-number" data-line-number="1552"></td>
        <td id="LC1552" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L1553" class="blob-num js-line-number" data-line-number="1553"></td>
        <td id="LC1553" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \param [in]     ch  Character to transmit.</span></td>
      </tr>
      <tr>
        <td id="L1554" class="blob-num js-line-number" data-line-number="1554"></td>
        <td id="LC1554" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L1555" class="blob-num js-line-number" data-line-number="1555"></td>
        <td id="LC1555" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \returns            Character to transmit.</span></td>
      </tr>
      <tr>
        <td id="L1556" class="blob-num js-line-number" data-line-number="1556"></td>
        <td id="LC1556" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1557" class="blob-num js-line-number" data-line-number="1557"></td>
        <td id="LC1557" class="blob-code blob-code-inner js-file-line">__STATIC_INLINE <span class="pl-c1">uint32_t</span> <span class="pl-smi">ITM_SendChar</span> (<span class="pl-c1">uint32_t</span> ch)</td>
      </tr>
      <tr>
        <td id="L1558" class="blob-num js-line-number" data-line-number="1558"></td>
        <td id="LC1558" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L1559" class="blob-num js-line-number" data-line-number="1559"></td>
        <td id="LC1559" class="blob-code blob-code-inner js-file-line">  <span class="pl-k">if</span> ((ITM-&gt;TCR &amp; ITM_TCR_ITMENA_Msk)                  &amp;&amp;      <span class="pl-c"><span class="pl-c">/*</span> ITM enabled <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1560" class="blob-num js-line-number" data-line-number="1560"></td>
        <td id="LC1560" class="blob-code blob-code-inner js-file-line">      (ITM-&gt;TER &amp; (<span class="pl-c1">1UL</span> &lt;&lt; <span class="pl-c1">0</span>)        )                    )     <span class="pl-c"><span class="pl-c">/*</span> ITM Port #0 enabled <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1561" class="blob-num js-line-number" data-line-number="1561"></td>
        <td id="LC1561" class="blob-code blob-code-inner js-file-line">  {</td>
      </tr>
      <tr>
        <td id="L1562" class="blob-num js-line-number" data-line-number="1562"></td>
        <td id="LC1562" class="blob-code blob-code-inner js-file-line">    <span class="pl-k">while</span> (ITM-&gt;PORT[<span class="pl-c1">0</span>].<span class="pl-smi">u32</span> == <span class="pl-c1">0</span>);</td>
      </tr>
      <tr>
        <td id="L1563" class="blob-num js-line-number" data-line-number="1563"></td>
        <td id="LC1563" class="blob-code blob-code-inner js-file-line">    ITM-&gt;PORT[<span class="pl-c1">0</span>].<span class="pl-smi">u8</span> = (<span class="pl-c1">uint8_t</span>) ch;</td>
      </tr>
      <tr>
        <td id="L1564" class="blob-num js-line-number" data-line-number="1564"></td>
        <td id="LC1564" class="blob-code blob-code-inner js-file-line">  }</td>
      </tr>
      <tr>
        <td id="L1565" class="blob-num js-line-number" data-line-number="1565"></td>
        <td id="LC1565" class="blob-code blob-code-inner js-file-line">  <span class="pl-k">return</span> (ch);</td>
      </tr>
      <tr>
        <td id="L1566" class="blob-num js-line-number" data-line-number="1566"></td>
        <td id="LC1566" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L1567" class="blob-num js-line-number" data-line-number="1567"></td>
        <td id="LC1567" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1568" class="blob-num js-line-number" data-line-number="1568"></td>
        <td id="LC1568" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1569" class="blob-num js-line-number" data-line-number="1569"></td>
        <td id="LC1569" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \brief  ITM Receive Character</span></td>
      </tr>
      <tr>
        <td id="L1570" class="blob-num js-line-number" data-line-number="1570"></td>
        <td id="LC1570" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L1571" class="blob-num js-line-number" data-line-number="1571"></td>
        <td id="LC1571" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    The function inputs a character via the external variable \ref ITM_RxBuffer.</span></td>
      </tr>
      <tr>
        <td id="L1572" class="blob-num js-line-number" data-line-number="1572"></td>
        <td id="LC1572" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L1573" class="blob-num js-line-number" data-line-number="1573"></td>
        <td id="LC1573" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \return             Received character.</span></td>
      </tr>
      <tr>
        <td id="L1574" class="blob-num js-line-number" data-line-number="1574"></td>
        <td id="LC1574" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \return         -1  No character pending.</span></td>
      </tr>
      <tr>
        <td id="L1575" class="blob-num js-line-number" data-line-number="1575"></td>
        <td id="LC1575" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1576" class="blob-num js-line-number" data-line-number="1576"></td>
        <td id="LC1576" class="blob-code blob-code-inner js-file-line">__STATIC_INLINE <span class="pl-c1">int32_t</span> <span class="pl-smi">ITM_ReceiveChar</span> (<span class="pl-k">void</span>) {</td>
      </tr>
      <tr>
        <td id="L1577" class="blob-num js-line-number" data-line-number="1577"></td>
        <td id="LC1577" class="blob-code blob-code-inner js-file-line">  <span class="pl-c1">int32_t</span> ch = -<span class="pl-c1">1</span>;                           <span class="pl-c"><span class="pl-c">/*</span> no character available <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1578" class="blob-num js-line-number" data-line-number="1578"></td>
        <td id="LC1578" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1579" class="blob-num js-line-number" data-line-number="1579"></td>
        <td id="LC1579" class="blob-code blob-code-inner js-file-line">  <span class="pl-k">if</span> (ITM_RxBuffer != ITM_RXBUFFER_EMPTY) {</td>
      </tr>
      <tr>
        <td id="L1580" class="blob-num js-line-number" data-line-number="1580"></td>
        <td id="LC1580" class="blob-code blob-code-inner js-file-line">    ch = ITM_RxBuffer;</td>
      </tr>
      <tr>
        <td id="L1581" class="blob-num js-line-number" data-line-number="1581"></td>
        <td id="LC1581" class="blob-code blob-code-inner js-file-line">    ITM_RxBuffer = ITM_RXBUFFER_EMPTY;       <span class="pl-c"><span class="pl-c">/*</span> ready for next character <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1582" class="blob-num js-line-number" data-line-number="1582"></td>
        <td id="LC1582" class="blob-code blob-code-inner js-file-line">  }</td>
      </tr>
      <tr>
        <td id="L1583" class="blob-num js-line-number" data-line-number="1583"></td>
        <td id="LC1583" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1584" class="blob-num js-line-number" data-line-number="1584"></td>
        <td id="LC1584" class="blob-code blob-code-inner js-file-line">  <span class="pl-k">return</span> (ch);</td>
      </tr>
      <tr>
        <td id="L1585" class="blob-num js-line-number" data-line-number="1585"></td>
        <td id="LC1585" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L1586" class="blob-num js-line-number" data-line-number="1586"></td>
        <td id="LC1586" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1587" class="blob-num js-line-number" data-line-number="1587"></td>
        <td id="LC1587" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1588" class="blob-num js-line-number" data-line-number="1588"></td>
        <td id="LC1588" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>* \brief  ITM Check Character</span></td>
      </tr>
      <tr>
        <td id="L1589" class="blob-num js-line-number" data-line-number="1589"></td>
        <td id="LC1589" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L1590" class="blob-num js-line-number" data-line-number="1590"></td>
        <td id="LC1590" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    The function checks whether a character is pending for reading in the variable \ref ITM_RxBuffer.</span></td>
      </tr>
      <tr>
        <td id="L1591" class="blob-num js-line-number" data-line-number="1591"></td>
        <td id="LC1591" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L1592" class="blob-num js-line-number" data-line-number="1592"></td>
        <td id="LC1592" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \return          0  No character available.</span></td>
      </tr>
      <tr>
        <td id="L1593" class="blob-num js-line-number" data-line-number="1593"></td>
        <td id="LC1593" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    \return          1  Character available.</span></td>
      </tr>
      <tr>
        <td id="L1594" class="blob-num js-line-number" data-line-number="1594"></td>
        <td id="LC1594" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1595" class="blob-num js-line-number" data-line-number="1595"></td>
        <td id="LC1595" class="blob-code blob-code-inner js-file-line">__STATIC_INLINE <span class="pl-c1">int32_t</span> <span class="pl-smi">ITM_CheckChar</span> (<span class="pl-k">void</span>) {</td>
      </tr>
      <tr>
        <td id="L1596" class="blob-num js-line-number" data-line-number="1596"></td>
        <td id="LC1596" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1597" class="blob-num js-line-number" data-line-number="1597"></td>
        <td id="LC1597" class="blob-code blob-code-inner js-file-line">  <span class="pl-k">if</span> (ITM_RxBuffer == ITM_RXBUFFER_EMPTY) {</td>
      </tr>
      <tr>
        <td id="L1598" class="blob-num js-line-number" data-line-number="1598"></td>
        <td id="LC1598" class="blob-code blob-code-inner js-file-line">    <span class="pl-k">return</span> (<span class="pl-c1">0</span>);                                 <span class="pl-c"><span class="pl-c">/*</span> no character available <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1599" class="blob-num js-line-number" data-line-number="1599"></td>
        <td id="LC1599" class="blob-code blob-code-inner js-file-line">  } <span class="pl-k">else</span> {</td>
      </tr>
      <tr>
        <td id="L1600" class="blob-num js-line-number" data-line-number="1600"></td>
        <td id="LC1600" class="blob-code blob-code-inner js-file-line">    <span class="pl-k">return</span> (<span class="pl-c1">1</span>);                                 <span class="pl-c"><span class="pl-c">/*</span>    character available <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1601" class="blob-num js-line-number" data-line-number="1601"></td>
        <td id="LC1601" class="blob-code blob-code-inner js-file-line">  }</td>
      </tr>
      <tr>
        <td id="L1602" class="blob-num js-line-number" data-line-number="1602"></td>
        <td id="LC1602" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L1603" class="blob-num js-line-number" data-line-number="1603"></td>
        <td id="LC1603" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1604" class="blob-num js-line-number" data-line-number="1604"></td>
        <td id="LC1604" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span>@} end of CMSIS_core_DebugFunctions <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1605" class="blob-num js-line-number" data-line-number="1605"></td>
        <td id="LC1605" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1606" class="blob-num js-line-number" data-line-number="1606"></td>
        <td id="LC1606" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">endif</span> <span class="pl-c"><span class="pl-c">/*</span> __CORE_CM3_H_DEPENDANT <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1607" class="blob-num js-line-number" data-line-number="1607"></td>
        <td id="LC1607" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1608" class="blob-num js-line-number" data-line-number="1608"></td>
        <td id="LC1608" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">endif</span> <span class="pl-c"><span class="pl-c">/*</span> __CMSIS_GENERIC <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L1609" class="blob-num js-line-number" data-line-number="1609"></td>
        <td id="LC1609" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1610" class="blob-num js-line-number" data-line-number="1610"></td>
        <td id="LC1610" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">ifdef</span> __cplusplus</td>
      </tr>
      <tr>
        <td id="L1611" class="blob-num js-line-number" data-line-number="1611"></td>
        <td id="LC1611" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L1612" class="blob-num js-line-number" data-line-number="1612"></td>
        <td id="LC1612" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">endif</span></td>
      </tr>
</table>

  </div>

</div>

<button type="button" data-facebox="#jump-to-line" data-facebox-class="linejump" data-hotkey="l" class="d-none">Jump to Line</button>
<div id="jump-to-line" style="display:none">
  <!-- '"` --><!-- </textarea></xmp> --></option></form><form accept-charset="UTF-8" action="" class="js-jump-to-line-form" method="get"><div style="margin:0;padding:0;display:inline"><input name="utf8" type="hidden" value="&#x2713;" /></div>
    <input class="form-control linejump-input js-jump-to-line-field" type="text" placeholder="Jump to line&hellip;" aria-label="Jump to line" autofocus>
    <button type="submit" class="btn">Go</button>
</form></div>

  </div>
  <div class="modal-backdrop js-touch-events"></div>
</div>




    </div>
  </div>

  </div>

      <div class="container site-footer-container">
  <div class="site-footer" role="contentinfo">
    <ul class="site-footer-links float-right">
        <li><a href="https://github.com/contact" data-ga-click="Footer, go to contact, text:contact">Contact GitHub</a></li>
      <li><a href="https://developer.github.com" data-ga-click="Footer, go to api, text:api">API</a></li>
      <li><a href="https://training.github.com" data-ga-click="Footer, go to training, text:training">Training</a></li>
      <li><a href="https://shop.github.com" data-ga-click="Footer, go to shop, text:shop">Shop</a></li>
        <li><a href="https://github.com/blog" data-ga-click="Footer, go to blog, text:blog">Blog</a></li>
        <li><a href="https://github.com/about" data-ga-click="Footer, go to about, text:about">About</a></li>

    </ul>

    <a href="https://github.com" aria-label="Homepage" class="site-footer-mark" title="GitHub">
      <svg aria-hidden="true" class="octicon octicon-mark-github" height="24" version="1.1" viewBox="0 0 16 16" width="24"><path fill-rule="evenodd" d="M8 0C3.58 0 0 3.58 0 8c0 3.54 2.29 6.53 5.47 7.59.4.07.55-.17.55-.38 0-.19-.01-.82-.01-1.49-2.01.37-2.53-.49-2.69-.94-.09-.23-.48-.94-.82-1.13-.28-.15-.68-.52-.01-.53.63-.01 1.08.58 1.23.82.72 1.21 1.87.87 2.33.66.07-.52.28-.87.51-1.07-1.78-.2-3.64-.89-3.64-3.95 0-.87.31-1.59.82-2.15-.08-.2-.36-1.02.08-2.12 0 0 .67-.21 2.2.82.64-.18 1.32-.27 2-.27.68 0 1.36.09 2 .27 1.53-1.04 2.2-.82 2.2-.82.44 1.1.16 1.92.08 2.12.51.56.82 1.27.82 2.15 0 3.07-1.87 3.75-3.65 3.95.29.25.54.73.54 1.48 0 1.07-.01 1.93-.01 2.2 0 .21.15.46.55.38A8.013 8.013 0 0 0 16 8c0-4.42-3.58-8-8-8z"/></svg>
</a>
    <ul class="site-footer-links">
      <li>&copy; 2017 <span title="0.32169s from github-fe-235f48c.cp1-iad.github.net">GitHub</span>, Inc.</li>
        <li><a href="https://github.com/site/terms" data-ga-click="Footer, go to terms, text:terms">Terms</a></li>
        <li><a href="https://github.com/site/privacy" data-ga-click="Footer, go to privacy, text:privacy">Privacy</a></li>
        <li><a href="https://github.com/security" data-ga-click="Footer, go to security, text:security">Security</a></li>
        <li><a href="https://status.github.com/" data-ga-click="Footer, go to status, text:status">Status</a></li>
        <li><a href="https://help.github.com" data-ga-click="Footer, go to help, text:help">Help</a></li>
    </ul>
  </div>
</div>



  

  <div id="ajax-error-message" class="ajax-error-message flash flash-error">
    <svg aria-hidden="true" class="octicon octicon-alert" height="16" version="1.1" viewBox="0 0 16 16" width="16"><path fill-rule="evenodd" d="M8.865 1.52c-.18-.31-.51-.5-.87-.5s-.69.19-.87.5L.275 13.5c-.18.31-.18.69 0 1 .19.31.52.5.87.5h13.7c.36 0 .69-.19.86-.5.17-.31.18-.69.01-1L8.865 1.52zM8.995 13h-2v-2h2v2zm0-3h-2V6h2v4z"/></svg>
    <button type="button" class="flash-close js-flash-close js-ajax-error-dismiss" aria-label="Dismiss error">
      <svg aria-hidden="true" class="octicon octicon-x" height="16" version="1.1" viewBox="0 0 12 16" width="12"><path fill-rule="evenodd" d="M7.48 8l3.75 3.75-1.48 1.48L6 9.48l-3.75 3.75-1.48-1.48L4.52 8 .77 4.25l1.48-1.48L6 6.52l3.75-3.75 1.48 1.48z"/></svg>
    </button>
    You can't perform that action at this time.
  </div>


    
    <script crossorigin="anonymous" integrity="sha256-9ZVgaDj6gInYIAHGtNp/a6M/Md3kgGXgA5AUO5DatkI=" src="https://assets-cdn.github.com/assets/frameworks-f595606838fa8089d82001c6b4da7f6ba33f31dde48065e00390143b90dab642.js"></script>
    <script async="async" crossorigin="anonymous" integrity="sha256-AvF1lnHqzBwcRSJpxr9Fq5ftyLKUeP5JfBDrmHAJBoI=" src="https://assets-cdn.github.com/assets/github-02f1759671eacc1c1c452269c6bf45ab97edc8b29478fe497c10eb9870090682.js"></script>
    
    
    
    
  <div class="js-stale-session-flash stale-session-flash flash flash-warn flash-banner d-none">
    <svg aria-hidden="true" class="octicon octicon-alert" height="16" version="1.1" viewBox="0 0 16 16" width="16"><path fill-rule="evenodd" d="M8.865 1.52c-.18-.31-.51-.5-.87-.5s-.69.19-.87.5L.275 13.5c-.18.31-.18.69 0 1 .19.31.52.5.87.5h13.7c.36 0 .69-.19.86-.5.17-.31.18-.69.01-1L8.865 1.52zM8.995 13h-2v-2h2v2zm0-3h-2V6h2v4z"/></svg>
    <span class="signed-in-tab-flash">You signed in with another tab or window. <a href="">Reload</a> to refresh your session.</span>
    <span class="signed-out-tab-flash">You signed out in another tab or window. <a href="">Reload</a> to refresh your session.</span>
  </div>
  <div class="facebox" id="facebox" style="display:none;">
  <div class="facebox-popup">
    <div class="facebox-content" role="dialog" aria-labelledby="facebox-header" aria-describedby="facebox-description">
    </div>
    <button type="button" class="facebox-close js-facebox-close" aria-label="Close modal">
      <svg aria-hidden="true" class="octicon octicon-x" height="16" version="1.1" viewBox="0 0 12 16" width="12"><path fill-rule="evenodd" d="M7.48 8l3.75 3.75-1.48 1.48L6 9.48l-3.75 3.75-1.48-1.48L4.52 8 .77 4.25l1.48-1.48L6 6.52l3.75-3.75 1.48 1.48z"/></svg>
    </button>
  </div>
</div>


  </body>
</html>

