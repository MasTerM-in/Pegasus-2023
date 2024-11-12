   >
        {/* Header and Navigation */}
        <header>
          <h1>My Website</h1>
          <nav>
            <ul>
              <li>
                <Link href="/">Home</Link>
              </li>
              <li>
                <Link href="/about">About</Link>
              </li>
              <li>
                <Link href="/contact">Contact</Link>
              </li>
            </ul>
          </nav>
        </header>

        {/* Main content of the page */}
        <main>{children}</main>

        {/* Footer */}
        <footer>
          <p>© 2024 My Website</p>
        </footer>
      </body>
    </html>
  );
}
